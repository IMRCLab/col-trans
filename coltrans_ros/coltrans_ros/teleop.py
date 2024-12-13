import numpy as np

import rclpy
from rclpy.node import Node

# from crazyflie_interfaces.msg import LogDataGeneric
# from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import Joy

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_aug')

        # self.cfs = ["cf5", "cf6"]

        self.setParamsServiceServer = self.create_client(SetParameters, "/crazyflie_server/set_parameters")

        self.setParamsServiceTeleop = self.create_client(SetParameters, "/teleop/set_parameters")

        # self.publisher = self.create_publisher(
                # PoseStamped, "{}/frontnet_targetpos_typed".format(self.cf), 10)

        self.timer = None
        self.buttons_prev = None

        self.subscription1 = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)


    def joy_callback(self, msg: Joy):
        buttons = np.array(msg.buttons)
        
        if self.buttons_prev is None:
            self.buttons_prev = buttons
            return

        if len(buttons) != len(self.buttons_prev):
            self.buttons_prev = buttons
            return

        buttonsChange = buttons - self.buttons_prev
        self.buttons_prev = buttons

        # 0 - green button
        if buttonsChange[0] == 1:
            # switch to manual teleoperation
            self.setParamTeleopString("mode", "cmd_vel_world")

        # 2 - blue button: switch to payload controller
        if buttonsChange[2] == 1:
            # switch to payload controller
            self.setParamInt("all.params.stabilizer.controller", 7)
            # self.setParamInt("all.params.usd.logging", 1)

            # # switch to manual teleoperation after some time
            # if self.timer is None:
            #     self.timer = self.create_timer(2.0, self.timer_callback)

        # 3 - yellow button
        if buttonsChange[3] == 1:
            # # switch to manual teleoperation after some time
            # if self.timer is None:
            #     self.timer = self.create_timer(0.5, self.timer_callback)

            self.setParamInt("all.params.usd.logging", 1)

        # 4 - LB
        if buttonsChange[4] == 1:
            # # switch back to "normal" regularization
            # self.setParamFloat("all.params.ctrlLeeP.lambda", 0.5)
            # self.setParamInt("all.params.ctrlLeeP.form_ctrl", 1)

            # switch desired formation off
            self.setParamInt("all.params.ctrlLeeP.form_ctrl", 0)

        # 5 - RB
        if buttonsChange[5] == 1:
            # switch to desired formation
            self.setParamFloat("all.params.ctrlLeeP.lambda", 4.0)
            self.setParamInt("all.params.ctrlLeeP.form_ctrl", 2)

        # land button: switch back to regular lee/mellinger controller!
        if buttonsChange[6] == 1:
            # stop logging
            self.setParamInt("all.params.usd.logging", 0)
            # make sure teleoperation is disabled
            self.setParamTeleopString("mode", "high_level")
            # switch to regular Lee controller
            self.setParamInt("all.params.stabilizer.controller", 2)


    def timer_callback(self):
        self.setParamTeleopString("mode", "cmd_vel_world")

        self.timer.destroy()
        self.timer = None

    def setParam(self, param_name, param_value):
        req = SetParameters.Request()
        req.parameters = [Parameter(name=param_name, value=param_value)]
        self.setParamsServiceServer.call_async(req)

    def setParamTeleopString(self, param_name, value):
        param_type = ParameterType.PARAMETER_STRING
        param_value = ParameterValue(type=param_type, string_value=str(value))
        req = SetParameters.Request()
        req.parameters = [Parameter(name=param_name, value=param_value)]
        self.setParamsServiceTeleop.call_async(req)

    def setParamInt(self, name, value):
        param_type = ParameterType.PARAMETER_INTEGER
        param_value = ParameterValue(type=param_type, integer_value=int(value))
        self.setParam(name, param_value)

    def setParamFloat(self, name, value):
        param_type = ParameterType.PARAMETER_DOUBLE
        param_value = ParameterValue(type=param_type, double_value=float(value))
        self.setParam(name, param_value)



def main(args=None):
    rclpy.init(args=args)

    node = TeleopNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()