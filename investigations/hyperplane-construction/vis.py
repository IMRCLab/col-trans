import numpy as np
import time
import rowan
import cvxpy as cp
import yaml
import typing
import argparse

# Meshcat
import meshcat as mc
import meshcat.geometry as mcg
import meshcat.transformations as mctf

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

# Moves plane with given n, such that it passes through p0
def plane_transform2(p0, n):

    R = mctf.identity_matrix()
    z_fixed = [0,0,1]
    z = n / np.linalg.norm(n)
    q = rowan.vector_vector_rotation(z_fixed, z)
    R1 = rowan.to_matrix(q)
    R[:3, 0] = R1[:,0]
    R[:3, 1] = R1[:,1]
    R[:3, 2] = R1[:,2]

    # project p0 onto the plane
    R[:3, 3] = p0

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

# computes the angle between two vectors
def vec_angle(v1, v2):
    return np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))


def skew(w):
    w = np.asarray(w).reshape(3,1)
    w1 = w[0,0]
    w2 = w[1,0]
    w3 = w[2,0]
    return np.array([[0, -w3, w2],[w3, 0, -w1],[-w2, w1, 0]]).reshape((3,3))

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
    parser = argparse.ArgumentParser()
    parser.add_argument("name")
    args = parser.parse_args()

    # load config
    with open("config.yaml", 'r') as ymlfile:
        cfg = yaml.safe_load(ymlfile)

    for c in cfg:
        if c["name"] == args.name:
            cfg = c
            break

    uav1 = UAV(**cfg["uavs"][0])
    uav2 = UAV(**cfg["uavs"][1])
    payload = Payload(**cfg["payload"])
    ppos = np.asarray([0,0,0])
    Fd = np.asarray(cfg["Fd"])
    Md = np.asarray(cfg["Md"])

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


    # point mass case

    # # optimization problem
    # lambda_svm = 1000

    # n = cp.Variable(3)
    # prob = cp.Problem(cp.Minimize(cp.sum_squares(n) + lambda_svm * (n.T @ Fd)**2 ),
    #     [
    #         n.T @ p1 <= -1,
    #         n.T @ p2 >= 1,
    #     ])
    # prob.solve()
    # n = n.value

    # rigid body case
    P = np.zeros((6, 6))
    P[0:3,0:3] = np.eye(3)
    P[0:3,3:6] = np.eye(3)
    P[3:6,0:3] = skew(p1a)
    P[3:6,3:6] = skew(p2a)
    print(P)
    # print(np.linalg.matrix_rank(P))
    P_inv = np.linalg.pinv(P)
    # P_inv = P.T @ np.linalg.inv(P@P.T)
    # print(P)

    print(P_inv)


    Rp = rowan.to_matrix(payload.rotation)
    R_blockdiag = np.zeros((6, 6))
    R_blockdiag[0:3,0:3] = Rp
    R_blockdiag[3:6,3:6] = Rp

    forces_at_attachment_points = R_blockdiag @ P_inv @ np.concatenate((Rp.T @ Fd, Md))

    Fd1 = forces_at_attachment_points[0:3]
    Fd2 = forces_at_attachment_points[3:6]

    # # optimization rather than Pinv
    # # see (22) in Lee's paper
    # Fd1 = cp.Variable(3)
    # Fd2 = cp.Variable(3)
    # prob = cp.Problem(cp.Minimize(
    #     cp.norm(Fd1) + cp.norm(Fd2) +
    #     cp.norm(skew(p1a) @ Rp.T @ Fd1 + skew(p2a) @ Rp.T @ Fd2 - Md)),
    #         [
    #             Fd1 + Fd2 == Fd,
    #             # skew(p1a) @ Rp.T @ Fd1 + skew(p2a) @ Rp.T @ Fd2 == Md,
    #         ])
    # prob.solve(verbose=False)
    # Fd1 = Fd1.value
    # Fd2 = Fd2.value



    # draw Fd
    vertices = np.array([p1a,p1a+Fd1*15]).T
    vis["Fd1"].set_object(mcg.Line(mcg.PointsGeometry(vertices), 
        material=mcg.LineBasicMaterial(linewidth=6, color=0xff0000)))

    vertices = np.array([p2a,p2a+Fd2*15]).T
    vis["Fd2"].set_object(mcg.Line(mcg.PointsGeometry(vertices), 
        material=mcg.LineBasicMaterial(linewidth=6, color=0x0000ff)))


    # optimization problem
    lambda_svm = 10
    Fd_scale = 1

    # original version
    # n = cp.Variable(3)
    # a = cp.Variable(1)
    # prob = cp.Problem(cp.Minimize(cp.sum_squares(n) + lambda_svm * (n.T @ (Fd_scale*(p1a+Fd1)) - a)**2 + lambda_svm * (n.T @ (Fd_scale*(p2a+Fd2)) - a)**2),
    #     [
    #         n.T @ p1  - a <= -1,
    #         n.T @ p1a - a <= -1,
    #         n.T @ p2  - a >= 1,
    #         n.T @ p2a - a >= 1,
    #     ])
    # prob.solve()

    # soft/hard-margin SVM version

    # lambda_svm = 100
    # n = cp.Variable(3)
    # a = cp.Variable(1)
    # slack1 = cp.Variable(1)
    # slack2 = cp.Variable(1)
    # prob = cp.Problem(cp.Minimize(cp.sum_squares(n) + lambda_svm * (slack1+slack2)),
    #     [
    #         n.T @ p1  - a <= -1,
    #         n.T @ p1a - a <= -1,
    #         n.T @ p2  - a >= 1,
    #         n.T @ p2a - a >= 1,
    #         n.T @ (p1a+Fd1) - a <= -1+slack1,
    #         n.T @ (p2a+Fd2) - a >= 1-slack2,
    #         slack1 >= 0,
    #         slack2 >= 0,
    #     ])
    # prob.solve()

    # pm 
    lambda_svm = 1000
    Fd_scale = 1

    pm = np.asarray(p1a) + (np.asarray(p2a)-np.asarray(p1a))/2

    n = cp.Variable(3)
    a = cp.Variable(1)
    prob = cp.Problem(cp.Minimize(cp.sum_squares(n) + lambda_svm * (n.T @ (Fd_scale*(pm+Fd1)) - a)**2 + lambda_svm * (n.T @ (Fd_scale*(pm+Fd2)) - a)**2),
        [
            n.T @ p1  - a <= -1,
            n.T @ p1a - a <= -1,
            n.T @ p2  - a >= 1,
            n.T @ p2a - a >= 1,
            n.T @ pm - a == 0,
        ])
    prob.solve()


    n = n.value
    a = a.value
    print(p1, p1a, p2, p2a)
    print(n,a)
    # exit()

    point0 = p1a
    point1 = p1a + np.cross(n, np.array([0,0,1]))

    # find the minimum intersection of plane and robot movement sphere
    intersection = plane_sphere_intersection(n, a, p1a, l1)
    if intersection is not None:
        center, radius = intersection
        print("intersection", center, radius)
        # compute the intersection point with the highest z-value
        v = project_point_on_plane(n, a, center + np.array([0,0,1])) - center
        print("v", v)
        v_normalized = v / np.linalg.norm(v)
        point2 = center + v_normalized * radius
        print(point2)

        n1 = np.cross(point0-point1, point0-point2)

        # and rotate it
        axis = np.cross(n1, np.array([0,0,1]))

        if uav1.safety_radius <= 2.0 * l1:
            angle1 = 2 * np.arcsin(uav1.safety_radius / (2 * l1))
            q1 = rowan.from_axis_angle(axis, -angle1)
            n1 = rowan.rotate(q1, n1)


    else:
        n1 = None


    point0 = p2a
    point1 = p2a + np.cross(n, np.array([0,0,1]))

    # find the minimum intersection of plane and robot movement sphere
    intersection = plane_sphere_intersection(n, a, p2a, l2)
    if intersection is not None:
        center, radius = intersection
        print("intersection", center, radius)
        # compute the intersection point with the highest z-value
        v = project_point_on_plane(n, a, center + np.array([0,0,1])) - center
        print("v", v)
        v_normalized = v / np.linalg.norm(v)
        point2 = center + v_normalized * radius
        print(point2)

        n2 = np.cross(point0-point1, point0-point2)

        # and rotate it
        axis = np.cross(n2, np.array([0,0,1]))

        if uav2.safety_radius <= 2.0 * l2:
            angle2 = 2 * np.arcsin(uav2.safety_radius / (2 * l2))
            q2 = rowan.from_axis_angle(axis, angle2)
            n2 = rowan.rotate(q2, n2)


    else:
        n2 = None



    # # tilt resulting hyperplanes (point mass case)
    # # now rotate the hyperplane in two directions to compute the two resulting hyperplanes
    # axis = np.cross(n, np.array([0,0,1]))

    # if uav1.safety_radius > 2.0 * l1:
    #     n1 = n
    # else:
    #     angle1 = 2 * np.arcsin(uav1.safety_radius / (2 * l1))
    #     q1 = rowan.from_axis_angle(axis, angle1)
    #     n1 = rowan.rotate(q1, n)

    # if uav2.safety_radius > 2.0 * l2:
    #     n2 = n
    # else:
    #     angle2 = 2 * np.arcsin(uav2.safety_radius / (2 * l2))
    #     q2 = rowan.from_axis_angle(axis, -angle2)
    #     n2 = -rowan.rotate(q2, n)

    # # compute per-robot hyperplanes using three points
    # point0 = p1a
    # point1 = p1a + np.cross(n1, np.array([0,0,1]))

    # # find the minimum intersection of plane and robot movement sphere
    # intersection = plane_sphere_intersection(n1, 0, p1a, l1)
    # if intersection is not None:
    #     center, radius = intersection
    #     print("intersection", center, radius)
    #     # compute the intersection point with the highest z-value
    #     v = project_point_on_plane(n1, 0, center + np.array([0,0,1])) - center
    #     print("v", v)
    #     v_normalized = v / np.linalg.norm(v)
    #     point2 = center + v_normalized * radius
    #     print(point2)

    #     n1 = np.cross(point0-point1, point0-point2)
    #     # n1 = n
    #     a1 = np.dot(n1, p1a)
    #     print(point0, point1, point2)
    # else:
    #     n1 = None

    # point0 = p2a
    # point1 = p2a + np.cross(n2, np.array([0,0,1]))

    # # find the minimum intersection of plane and robot movement sphere
    # intersection = plane_sphere_intersection(n2, 0, p2a, l2)
    # if intersection is not None:
    #     center, radius = intersection
    #     print("intersection", center, radius)
    #     # compute the intersection point with the highest z-value
    #     v = project_point_on_plane(n2, 0, center + np.array([0,0,1])) - center
    #     print("v", v)
    #     v_normalized = v / np.linalg.norm(v)
    #     point2 = center + v_normalized * radius
    #     print(point2)

    #     n2 = np.cross(point0-point1, point0-point2)
    #     # n1 = n
    #     a2 = np.dot(n2, p2a)
    #     print(point0, point1, point2)
    # else:
    #     n2 = None



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
    vis["hp"].set_transform(plane_transform(ppos, n, a))

    if n1 is not None:
        hp = mcg.Mesh(Plane(), 
                                material=mcg.MeshBasicMaterial(
                                    opacity=1.0,
                                    color=0xFF0000))
        vis["hp1"].set_object(hp)
        vis["hp1"].set_transform(plane_transform2(p1a, n1))

    if n2 is not None:
        hp = mcg.Mesh(Plane(), 
                                material=mcg.MeshBasicMaterial(
                                    opacity=1.0,
                                    color=0x0000FF))
        vis["hp2"].set_object(hp)
        vis["hp2"].set_transform(plane_transform2(p2a, n2))


    vis.open()

    time.sleep(100)


if __name__ == '__main__':
    main()


