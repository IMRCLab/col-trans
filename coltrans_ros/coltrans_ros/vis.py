from functools import partial
import numpy as np
import rowan

import rclpy
from rclpy.node import Node

from crazyflie_interfaces.msg import LogDataGeneric

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException

# Meshcat
import meshcat as mc
import meshcat.geometry as mcg
import meshcat.transformations as mctf


# copied from https://github.com/rdeits/meshcat-python/blob/master/src/meshcat/geometry.py#L83
# since the latest pip-version doesn't include it yet
class Plane(mcg.Geometry):

    def __init__(self, material=mcg.MeshPhongMaterial(), width=1, height=1, widthSegments=1, heightSegments=1):
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
    if np.linalg.norm(n) < 1e-6:
        return R
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

class VisualizationNode(Node):

    def __init__(self):
        super().__init__('vis')

        print(self.get_topic_names_and_types())

        cfs = ['cf5', 'cf6']
        self.cfs = cfs
        
        # initalize meshcat
        self.vis = mc.Visualizer()
        self.vis.open()

        self.vis["/Cameras/default"].set_transform(
            mctf.translation_matrix([0, 0, 0]).dot(
            mctf.euler_matrix(0, np.radians(-30), np.radians(-180))))

        self.vis["/Cameras/default/rotated/<object>"].set_transform(
            mctf.translation_matrix([2, 0, 0]))

        # for each crazyflie, generate a 3D model, a sphere, and a hyperplane
        for cf in cfs:
            model = mcg.StlMeshGeometry.from_file("/home/whoenig/projects/tuberlin/col-trans/sim/Animator/cf2_assembly.stl")
            self.vis["{}_model".format(cf)].set_object(model)

            sphere = mcg.Mesh(mcg.Sphere(0.1), 
                            material=mcg.MeshBasicMaterial(
                                opacity=0.05,
                                color=0x000000))
            self.vis["{}_sphere".format(cf)].set_object(sphere)

            hp = mcg.Mesh(Plane(), 
                            material=mcg.MeshBasicMaterial(
                                opacity=0.5,
                                color=0xff0000))
            self.vis["{}_hp".format(cf)].set_object(hp)

            hp2 = mcg.Mesh(Plane(), 
                            material=mcg.MeshBasicMaterial(
                                opacity=0.5,
                                color=0xff0000))
            self.vis["{}_hp2".format(cf)].set_object(hp2)


        # for the payload set
        # payload = mcg.Mesh(mcg.Sphere(0.02),
        #     mcg.MeshBasicMaterial(color=0xff11dd))
        # self.vis["payload"].set_object(payload)

        model = mcg.StlMeshGeometry.from_file("/home/whoenig/projects/tuberlin/col-trans/sim/Animator/Triangle_meteres_centered.stl")
        self.vis["payload".format(cf)].set_object(model)

        # subscribe to ROS2 topics for data
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.subscriptions_ = []
        for cf in cfs:
            subscription = self.create_subscription(
                LogDataGeneric,
                '{}/ctrlLeeP'.format(cf),
                partial(self.listener_callback, name=cf),
                10)
            self.subscriptions_.append(subscription)

            subscription = self.create_subscription(
                LogDataGeneric,
                '{}/ctrlLeeP2'.format(cf),
                partial(self.listener_callback2, name=cf),
                10)
            self.subscriptions_.append(subscription)


    def timer_callback(self):
        try:
            for cf in self.cfs:
                trans = self.tf_buffer.lookup_transform("world", cf, rclpy.time.Time())
                pos = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
                # print(cf, pos)
                self.vis["{}_model".format(cf)].set_transform(mctf.translation_matrix(pos))
                self.vis["{}_sphere".format(cf)].set_transform(mctf.translation_matrix(pos))

            # update the payload
            trans = self.tf_buffer.lookup_transform("world", "payload", rclpy.time.Time())
            pos = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            quat = np.array([trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z])
            # use an additional pi rotation, since the model is 180deg different than the defined orientation in mocap
            self.vis["payload"].set_transform(mctf.translation_matrix(pos).dot(mctf.quaternion_matrix(quat).dot(mctf.euler_matrix(0,0,np.pi))))

        except LookupException as e:
            self.get_logger().error('failed to get transform {} \n'.format(repr(e)))

    def listener_callback(self, msg: LogDataGeneric, name: str):
        # the expected configuration is
        # vars: ["ctrlLeeP.n1x", "ctrlLeeP.n1y", "ctrlLeeP.n1z", "ctrlLeeP.desVirtInpx", , "ctrlLeeP.desVirtInpy", , "ctrlLeeP.desVirtInpz"]

        # self.get_logger().info('I heard: "%s"' % msg)

        # draw hyperplane
        try:

            trans = self.tf_buffer.lookup_transform("world", "payload", rclpy.time.Time())
            ppos = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        except LookupException as e:
            self.get_logger().error('failed to get transform {} \n'.format(repr(e)))
            return

        n = np.array(msg.values[0:3])
        a = np.dot(n, ppos)

        R = plane_transform(ppos, n, a)
        self.vis["{}_hp".format(name)].set_transform(R)

        # draw normal
        vertices = np.array([ppos,ppos+n]).T
        self.vis["{}_hpn".format(name)].set_object(mcg.Line(
            mcg.PointsGeometry(vertices),
            material=mcg.LineBasicMaterial(linewidth=2, color=0xff0000, opacity=0.1)))

        # draw mu
        # normalize mu because they are very small in values
        mu = np.array(msg.values[3:6])
        # normmu = np.linalg.norm(mu)
        vertices = np.array([ppos,ppos+mu*15]).T
        self.vis["{}_mu".format(name)].set_object(mcg.Line(mcg.PointsGeometry(vertices), 
            material=mcg.LineBasicMaterial(linewidth=6, color=0x0000ff)))


    def listener_callback2(self, msg: LogDataGeneric, name: str):
        # the expected configuration is
        # vars: ["ctrlLeeP.n1x", "ctrlLeeP.n1y", "ctrlLeeP.n1z", "ctrlLeeP.desVirtInpx", , "ctrlLeeP.desVirtInpy", , "ctrlLeeP.desVirtInpz"]

        # self.get_logger().info('I heard: "%s"' % msg)

        # draw hyperplane
        try:

            trans = self.tf_buffer.lookup_transform("world", "payload", rclpy.time.Time())
            ppos = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        except LookupException as e:
            self.get_logger().error('failed to get transform {} \n'.format(repr(e)))
            return

        n = np.array(msg.values[0:3])
        a = np.dot(n, ppos)

        R = plane_transform(ppos, n, a)
        self.vis["{}_hp".format(name)].set_transform(R)

        self.vis["{}_hp2".format(name)].set_transform(R)

        # draw normal
        vertices = np.array([ppos,ppos+n]).T
        self.vis["{}_hp2n".format(name)].set_object(mcg.Line(
            mcg.PointsGeometry(vertices),
            material=mcg.LineBasicMaterial(linewidth=2, color=0xff0000, opacity=0.1)))

        # draw Fd
        # normalize mu because they are very small in values
        Fd = np.array(msg.values[3:6])
        # normmu = np.linalg.norm(mu)
        vertices = np.array([ppos,ppos+Fd*15]).T
        self.vis["{}_Fd".format(name)].set_object(mcg.Line(mcg.PointsGeometry(vertices), 
            material=mcg.LineBasicMaterial(linewidth=6, color=0x00ff00)))



def main(args=None):
    rclpy.init(args=args)

    node = VisualizationNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()