import numpy as np
import time
import rowan
import cvxpy as cp

# Meshcat
import meshcat as mc
import meshcat.geometry as mcg
import meshcat.transformations as mctf

# cable length [m], inclination [deg, 0-180], azimuth [deg, 0-360]
uav1 = [0.6, 20, 0]
uav2 = [0.9, -20, 45]

optimization = True

# uav1 = [0.5, 10, 0]
# uav2 = [0.9, -10, 0]

safety_radius = 0.15

ppos = np.array([0,0,0.0])
Fd = np.array([0.0,0.0,0.1])

def sphericalToCartCoord(spherical, use_degrees=True):
    r, inc, azi = spherical
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
# The plane fulfills n . p + a = 0
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
    p_on_plane = p0 - (np.dot(p0, z) + a) * z
    # print(np.dot(z, p_on_plane) + a)
    R[:3, 3] = p_on_plane

    return R

def main():
    vis = mc.Visualizer()

    l1 = uav1[0]
    l2 = uav2[0]

    vis["workspace1"].set_object(mcg.Mesh(mcg.Sphere(l1),
                            material=mcg.MeshLambertMaterial(opacity=0.1, color=0xFF0000)))

    vis["workspace2"].set_object(mcg.Mesh(mcg.Sphere(l2),
                            material=mcg.MeshLambertMaterial(opacity=0.1, color=0x0000FF)))

    # draw Fd
    # normalize mu because they are very small in values
    # normmu = np.linalg.norm(mu)
    vertices = np.array([ppos,ppos+Fd*15]).T
    vis["Fd"].set_object(mcg.Line(mcg.PointsGeometry(vertices), 
        material=mcg.LineBasicMaterial(linewidth=6, color=0x00ff00)))

    # draw UAVs (as spheres)

    p1 = ppos + sphericalToCartCoord(uav1)

    vis["uav1"].set_object(mcg.Mesh(mcg.Sphere(safety_radius),
                            material=mcg.MeshLambertMaterial(opacity=1.0, color=0xFF0000)))
    vis["uav1"].set_transform(mctf.translation_matrix(p1))

    p2 = ppos + sphericalToCartCoord(uav2)

    vis["uav2"].set_object(mcg.Mesh(mcg.Sphere(safety_radius),
                            material=mcg.MeshLambertMaterial(opacity=1.0, color=0x0000FF)))
    vis["uav2"].set_transform(mctf.translation_matrix(p2))

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



    # tilt resulting hyperplanes
    axis = np.cross(n, np.array([0,0,1]))
    angle1 = np.arcsin(safety_radius / l1)
    q1 = rowan.from_axis_angle(axis, angle1)
    n1 = rowan.rotate(q1, n)

    angle2 = np.arcsin(safety_radius / l2)
    q2 = rowan.from_axis_angle(axis, -angle2)
    n2 = rowan.rotate(q2, n)

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

    hp = mcg.Mesh(Plane(), 
                            material=mcg.MeshBasicMaterial(
                                opacity=1.0,
                                color=0xFF0000))
    vis["hp1"].set_object(hp)
    vis["hp1"].set_transform(plane_transform(ppos, n1, 0))

    hp = mcg.Mesh(Plane(), 
                            material=mcg.MeshBasicMaterial(
                                opacity=1.0,
                                color=0x0000FF))
    vis["hp2"].set_object(hp)
    vis["hp2"].set_transform(plane_transform(ppos, n2, 0))


    vis.open()

    time.sleep(100)


if __name__ == '__main__':
    main()