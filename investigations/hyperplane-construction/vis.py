import numpy as np
import time
import rowan
import cvxpy as cp
import yaml
import typing

# Meshcat
import meshcat as mc
import meshcat.geometry as mcg
import meshcat.transformations as mctf

optimization = False

def sphericalToCartCoord(r, inc, azi, use_degrees=True):
    # r, inc, azi = spherical
    if use_degrees:
        inc = np.radians(inc)
        azi = np.radians(azi)

    return r * np.array([np.sin(inc)*np.cos(azi), np.sin(inc)*np.sin(azi), np.cos(inc)])

# copied from https://github.com/rdeits/meshcat-python/blob/master/src/meshcat/geometry.py#L83
# since the latest pip-version doesn't include it yet
class Plane(mcg.Geometry):

    def __init__(self, material=mcg.MeshPhongMaterial(), width=2, height=2, widthSegments=1, heightSegments=1):
        super(Plane, self).__init__()
        self.width = width
        self.height = height
        self.widthSegments = widthSegments
        self.heightSegments = heightSegments
        self.material = material

    def lower(self, object_data):
        return {
            u"uuid": self.uuid,
            u"type": u"PlaneGeometry",
            u"width": self.width,
            u"height": self.height,
            u"widthSegments": self.widthSegments,
            u"heightSegments": self.heightSegments,
        }

# Compute the transformation for a plane with given norman n and offset a
# to be shown as close as possible to p0
# The plane fulfills n . p - a = 0
def plane_transform(p0, n, a):

    R = mctf.identity_matrix()
    z_fixed = [0,0,1]
    z = n / np.linalg.norm(n)
    q = rowan.vector_vector_rotation(z_fixed, z)
    R1 = rowan.to_matrix(q)
    R[:3, 0] = R1[:,0]
    R[:3, 1] = R1[:,1]
    R[:3, 2] = R1[:,2]

    # project p0 onto the plane
    p_on_plane = project_point_on_plane(n, a, p0)
    R[:3, 3] = p_on_plane

    return R

# projects point p onto plane (given by n.p = a)
def project_point_on_plane(n, a, p):
    # compute signed distance sphere -> hyperplane
    length = np.linalg.norm(n)
    dist = (np.dot(n,p) - a) / length
    p_on_plane = p - dist * n / length
    return p_on_plane


# Computes the circle that is created when intersecting a plane given as (n.p = a)
# with a sphere at center c with radius r
# See https://math.stackexchange.com/questions/943383/determine-circle-of-intersection-of-plane-and-sphere
def plane_sphere_intersection(n, a, c, r):
    # compute signed distance sphere -> hyperplane
    length = np.linalg.norm(n)
    dist = (np.dot(n,c) - a) / length

    # if the minimum distance is greater than r, there is no intersection
    if np.abs(dist) > r:
        return None

    # otherwise, compute the center point (on the plane) and radius of the resulting circle
    center = c - dist * n / length
    radius = np.sqrt(r**2 - dist**2)

    return center, radius

class UAV(typing.NamedTuple):
    cable_length: float # m
    inclination: float # deg
    azimuth: float # deg
    safety_radius: float # m
    attached: int

class Payload(typing.NamedTuple):
    model: str
    rotation: list[int]
    attachementpoints: list[list[float]]

def main():
    # load config
    with open("config.yaml", 'r') as ymlfile:
        cfg = yaml.safe_load(ymlfile)

    cfg = cfg[1]

    uav1 = UAV(**cfg["uavs"][0])
    uav2 = UAV(**cfg["uavs"][1])
    payload = Payload(**cfg["payload"])
    ppos = np.asarray([0,0,0])
    Fd = np.asarray(cfg["Fd"])

    l1 = uav1.cable_length
    l2 = uav2.cable_length


    vis = mc.Visualizer()


    # draw Fd
    # normalize mu because they are very small in values
    # normmu = np.linalg.norm(mu)
    vertices = np.array([ppos,ppos+Fd*15]).T
    vis["Fd"].set_object(mcg.Line(mcg.PointsGeometry(vertices), 
        material=mcg.LineBasicMaterial(linewidth=6, color=0x00ff00)))

    # draw UAVs (as spheres)
    p1a = payload.attachementpoints[uav1.attached]
    p1 = p1a + sphericalToCartCoord(uav1.cable_length, uav1.inclination, uav1.azimuth)

    vis["workspace1"].set_object(mcg.Mesh(mcg.Sphere(l1),
                            material=mcg.MeshLambertMaterial(opacity=0.1, color=0xFF0000)))
    vis["workspace1"].set_transform(mctf.translation_matrix(p1a))
    
    vis["uav1"].set_object(mcg.Mesh(mcg.Sphere(uav1.safety_radius),
                            material=mcg.MeshLambertMaterial(opacity=1.0, color=0xFF0000)))
    vis["uav1"].set_transform(mctf.translation_matrix(p1))

    p2a = payload.attachementpoints[uav2.attached]
    p2 = p2a + sphericalToCartCoord(uav2.cable_length, uav2.inclination, uav2.azimuth)

    vis["workspace2"].set_object(mcg.Mesh(mcg.Sphere(l2),
                            material=mcg.MeshLambertMaterial(opacity=0.1, color=0x0000FF)))
    vis["workspace2"].set_transform(mctf.translation_matrix(p2a))

    vis["uav2"].set_object(mcg.Mesh(mcg.Sphere(uav2.safety_radius),
                            material=mcg.MeshLambertMaterial(opacity=1.0, color=0x0000FF)))
    vis["uav2"].set_transform(mctf.translation_matrix(p2))

    # draw payload
    if payload.model != "pointmass":
        vis["payload"].set_object(mcg.StlMeshGeometry.from_file(payload.model))

    # draw cables

    vertices = np.array([p1a, p1]).T
    vis["cable1"].set_object(mcg.Line(mcg.PointsGeometry(vertices), 
        material=mcg.LineBasicMaterial(linewidth=6, color=0xffffff)))

    vertices = np.array([p2a, p2]).T
    vis["cable2"].set_object(mcg.Line(mcg.PointsGeometry(vertices), 
        material=mcg.LineBasicMaterial(linewidth=6, color=0xffffff)))

    if optimization:
        # approach 1: use a QP to find the initial hyperplane

        # cvxpy
        # distance point x to hyperplane (n,b) is: d: (n.x - b) / norm(n)
        # so we have
        # (n1 . p1 - b1) / |n1| >= safety_radius
        # (n1 . p2 - b1) / |n1| <= -safety_radius

        n = cp.Variable(3)
        prob = cp.Problem(cp.Minimize(cp.norm(n) + 10*(n.T @ Fd)**2),
                    [
                        n.T @ p1 <= -1,
                    n.T @ p2 >= 1,
                    #   cp.norm(n) >= safety_radius,
                    ])

        # prob = cp.Problem(cp.Minimize(cp.norm(n) + 50000*(n.T @ Fd)**2),
        #              [
        #                 n.T @ p1 >= safety_radius * np.sqrt(3),
        #               n.T @ p2 <= -safety_radius * np.sqrt(3),
        #               # Maximum norm: sqrt(3)
        #               n >= -1,
        #                 n <= 1,
        #               ])

        prob.solve()
        print(prob.value)
        print(n.value, Fd, (n.value.T @ Fd) / np.linalg.norm(n.value))
        print(n.value, p1, (n.value.T @ p1) / np.linalg.norm(n.value))
        print(n.value, p2, (n.value.T @ p2) / np.linalg.norm(n.value))
        n = n.value

    else:
        # approach 2: use geometric reasoning to find the initial hyperplane

        # Basic version: use hyperplane that contains Fd and
        # vector perpendicular to p2-p1
        v1 = p2 - p1
        # v1 = v1 / np.linalg.norm(v1)
        v2 = np.array([0,0,1])
        v3 = np.cross(v1, v2)

        n = np.cross(Fd, v3)
        # n = n / np.linalg.norm(n)
        print(n)

        # check if hyperplane is between UAVs
        n_normalized = n / np.linalg.norm(n)
        d1 = np.dot(n_normalized, p1)
        d2 = np.dot(n_normalized, p2)
        print(d1, d2)
        if d1 > 0 or d2 < 0:
            print("hyperplane not between UAVs!")

            # project p1 on plane
            p1_proj = p1 - d1 * n_normalized
            p2_proj = p2 - d2 * n_normalized

            v1_proj = p2_proj - p1_proj
            v1_proj_normalized = v1_proj / np.linalg.norm(v1_proj)

            axis = np.cross(n, v1)
            d = np.linalg.norm((p1 - ppos) - (v1_proj_normalized*l1 - ppos))
            print(d)
            angle = 2 * np.arcsin(d / l1) + np.arcsin(safety_radius / l1)

            q = rowan.from_axis_angle(axis, angle)
            n = rowan.rotate(q, n)



    # tilt resulting hyperplanes (point mass case)
    axis = np.cross(n, np.array([0,0,1]))

    if np.linalg.norm(axis) > 1e-6:
        angle1 = np.arcsin(uav1.safety_radius / l1)
        q1 = rowan.from_axis_angle(axis, angle1)
        n1 = rowan.rotate(q1, n)
        a1 = 0

        angle2 = np.arcsin(uav2.safety_radius / l2)
        q2 = rowan.from_axis_angle(axis, -angle2)
        n2 = rowan.rotate(q2, n)
        a2 = 0
    else:
        n1 = None
        n2 = None

    # compute per-robot hyperplanes using three points
    point0 = p1a
    point1 = p1a + np.cross(n1, np.array([0,0,1]))

    # find the minimum intersection of plane and robot movement sphere
    intersection = plane_sphere_intersection(n1, 0, p1a, l1)
    if intersection is not None:
        center, radius = intersection
        print("intersection", center, radius)
        # compute the intersection point with the highest z-value
        v = project_point_on_plane(n1, 0, center + np.array([0,0,1])) - center
        print("v", v)
        v_normalized = v / np.linalg.norm(v)
        point2 = center + v_normalized * radius
        print(point2)

        n1 = np.cross(point0-point1, point0-point2)
        # n1 = n
        a1 = np.dot(n1, p1a)
        print(point0, point1, point2)
    else:
        n1 = None

    point0 = p2a
    point1 = p2a + np.cross(n2, np.array([0,0,1]))

    # find the minimum intersection of plane and robot movement sphere
    intersection = plane_sphere_intersection(n2, 0, p2a, l2)
    if intersection is not None:
        center, radius = intersection
        print("intersection", center, radius)
        # compute the intersection point with the highest z-value
        v = project_point_on_plane(n2, 0, center + np.array([0,0,1])) - center
        print("v", v)
        v_normalized = v / np.linalg.norm(v)
        point2 = center + v_normalized * radius
        print(point2)

        n2 = np.cross(point0-point1, point0-point2)
        # n1 = n
        a2 = np.dot(n2, p2a)
        print(point0, point1, point2)
    else:
        n2 = None



    # point2 = 

    # w points
    # p0
    # any point between p1 to p2: pm(t) = p1 + t*(p2 - p1); t=0...1
    # Fd

    # basic idea
    # first use geometric construction (that includes Fd)
    # if that doesn't fulfill the hyperplane constraints, switch to SVM formulation (see above) -OR-
    # switch to original geometric solution
    # "tilt" the resulting hyperplane two ways to construct the two needed hyperplanes adjusting for the robot size
    # tilting: 
    #   a) rotate n around axis of [0,0,1]xn
    #   b) angle is defined by 2 arcsin(safety_radius/l)

    # how to deal with rigid bodies?

    # draw computed hyperplanes
    hp = mcg.Mesh(Plane(), 
                            material=mcg.MeshBasicMaterial(
                                opacity=1.0,
                                color=0x00FF00))
    vis["hp"].set_object(hp)
    vis["hp"].set_transform(plane_transform(ppos, n, 0))

    if n1 is not None:
        hp = mcg.Mesh(Plane(), 
                                material=mcg.MeshBasicMaterial(
                                    opacity=1.0,
                                    color=0xFF0000))
        vis["hp1"].set_object(hp)
        vis["hp1"].set_transform(plane_transform(p1a, n1, a1))

    if n2 is not None:
        hp = mcg.Mesh(Plane(), 
                                material=mcg.MeshBasicMaterial(
                                    opacity=1.0,
                                    color=0x0000FF))
        vis["hp2"].set_object(hp)
        vis["hp2"].set_transform(plane_transform(p2a, n2, a2))


    vis.open()

    time.sleep(100)


if __name__ == '__main__':
    main()