from functools import partial
import numpy as np
import rowan

import rclpy
from rclpy.node import Node

from crazyflie_interfaces.msg import LogDataGeneric

class VisualizationNode(Node):

    def __init__(self):
        super().__init__('vis')

        print(self.get_topic_names_and_types())

        cfs = ['cf5']
        self.cfs = cfs
        self.last_q = None
        self.last_timestamp = None
        
        self.subscriptions_ = []
        for cf in cfs:
            subscription = self.create_subscription(
                LogDataGeneric,
                '{}/payloadDbg'.format(cf),
                partial(self.listener_callback, name=cf),
                10)
            self.subscriptions_.append(subscription)


    def listener_callback(self, msg: LogDataGeneric, name: str):
        # the expected configuration is
        # vars: ["stateEstimate.pqx", "stateEstimate.pqy", "stateEstimate.pqz", "stateEstimate.pwx", "stateEstimate.pwy", "stateEstimate.pwz"]

        # assume quaternion is normalized
        qw = np.sqrt(1 - msg.values[0]**2 - msg.values[1]**2 - msg.values[2]**2)
        q = np.array([msg.values[0], msg.values[1], msg.values[2], qw])
        q = rowan.normalize(q)

        if self.last_q is None:
            self.last_q = q
            self.last_timestamp = msg.timestamp
            return

        omega = msg.values[3:6]


        rpy = rowan.to_euler(q, "xyz")
        print(np.degrees(rpy), omega)

        dt = (msg.timestamp - self.last_timestamp)/1000
        print(dt)
        q_next = rowan.calculus.integrate(self.last_q, omega, dt)

        print(q, q_next, rowan.allclose(q, q_next), rowan.geometry.distance(q, q_next))

        # self.get_logger().info('I heard: ' % msg)

        self.last_q = q
        self.last_timestamp = msg.timestamp



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