#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from hapthexa_msgs.msg import ForceSensor
from hapthexa_msgs.msg import Empty

import math

class LowPass:
    def __init__(self, a):
        self._a = 1.0 if a > 1.0 else 0.0 if a < 0.0 else a
        self._previous_ret = 0.0

    def lpf(self, value):
        ret = self._a * self._previous_ret + (1.0-self._a) * value
        self._previous_ret = ret  
        return ret

class ForceSensorCorrection(Node):

    def __init__(self):
        super().__init__('force_sensor')
        self._force_sensor_raw_sub = self.create_subscription(ForceSensor, 'force_sensor_raw', self.force_sensor_raw_sub_callback,10)
        self._force_sensor_pub = self.create_publisher(ForceSensor, 'force_sensor', 10)

        self._force_sensor_raw_sub = self.create_subscription(Empty, 'calibrate_signal', self.calibrate_signal_sub_callback,10)

        self.declare_parameter("loadcell1_zero", 127)
        self.declare_parameter("loadcell2_zero", 127)
        self.declare_parameter("piezo_zero", 0)
        self.declare_parameter("loadcell1_max", 255)
        self.declare_parameter("loadcell2_max", 255)
        self.declare_parameter("loadcell1_min", -127)
        self.declare_parameter("loadcell2_min", -127)
        self.declare_parameter("loadcell1_flag", 1)
        self.declare_parameter("loadcell2_flag", 1)

        self._filter_loadcell1 = LowPass(0.7)
        self._filter_loadcell2 = LowPass(0.7) 
        self._filter_piezo = LowPass(0.7) 

        self._loadcell1_raw = 0
        self._loadcell2_raw = 0
        self._piezo_raw = 0

    def force_sensor_raw_sub_callback(self, msg):
        send_msg = ForceSensor()
        send_msg.loadcell1_raw = msg.loadcell1_raw
        send_msg.loadcell2_raw = msg.loadcell2_raw
        send_msg.piezo_raw = msg.piezo_raw

        self._loadcell1_raw = msg.loadcell1_raw
        self._loadcell2_raw = msg.loadcell2_raw
        self._piezo_raw = msg.piezo_raw

        loadcell1 = (self.get_parameter("loadcell1_flag").get_parameter_value().integer_value * (msg.loadcell1_raw - self.get_parameter("loadcell1_zero").get_parameter_value().integer_value))
        loadcell2 = (self.get_parameter("loadcell2_flag").get_parameter_value().integer_value * (msg.loadcell2_raw - self.get_parameter("loadcell2_zero").get_parameter_value().integer_value))
        if loadcell1 > 0:
            loadcell1 /= abs(self.get_parameter("loadcell1_max").get_parameter_value().integer_value)
        else:
            loadcell1 /= abs(self.get_parameter("loadcell1_min").get_parameter_value().integer_value)
        if loadcell2 > 0:
            loadcell2 /= abs(self.get_parameter("loadcell2_max").get_parameter_value().integer_value)
        else:
            loadcell2 /= abs(self.get_parameter("loadcell2_min").get_parameter_value().integer_value)

        send_msg.loadcell1 = self._filter_loadcell1.lpf(1.0 if loadcell1 > 1.0 else -1.0 if loadcell1 < -1.0 else loadcell1)
        send_msg.loadcell2 = self._filter_loadcell2.lpf(1.0 if loadcell2 > 1.0 else -1.0 if loadcell2 < -1.0 else loadcell2)

        send_msg.radial_direction = math.atan2(send_msg.loadcell2, send_msg.loadcell1)
        send_msg.radial_magnitude = math.hypot(send_msg.loadcell2, send_msg.loadcell1)

        send_msg.piezo = self._filter_piezo.lpf((msg.piezo_raw - self.get_parameter("piezo_zero").get_parameter_value().integer_value)/255)

        send_msg.x = True if abs(loadcell1) > 0.5 else False
        send_msg.y = True if abs(loadcell2) > 0.5 else False
        send_msg.z = True if send_msg.piezo > 0.5 else False

        self._force_sensor_pub.publish(send_msg)

    def calibrate_signal_sub_callback(self, msg):
        self.get_logger().warn('calibrate signal catched')
        self.set_parameters([rclpy.parameter.Parameter("loadcell1_zero", rclpy.Parameter.Type.INTEGER, self._loadcell1_raw)])
        self.set_parameters([rclpy.parameter.Parameter("loadcell2_zero", rclpy.Parameter.Type.INTEGER, self._loadcell2_raw)])

def main(args=None):
    rclpy.init(args=args)
    force_sensor = ForceSensorCorrection()
    rclpy.spin(force_sensor)
    force_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()