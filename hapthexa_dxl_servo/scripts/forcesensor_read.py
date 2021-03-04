#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from hapthexa_msgs.msg import ForceSensor
from hapthexa_msgs.msg import Attitude

from math import pi

import serial, serial.tools.list_ports
import time

class ForcesensorRead(Node):

    def __init__(self):
        super().__init__('forcesensor_read')
        self.front_left_pub_ = self.create_publisher(ForceSensor, 'hapthexa/leg/front_left/force_sensor_raw', 10)
        self.middle_left_pub_ = self.create_publisher(ForceSensor, 'hapthexa/leg/middle_left/force_sensor_raw', 10)
        self.rear_left_pub_ = self.create_publisher(ForceSensor, 'hapthexa/leg/rear_left/force_sensor_raw', 10)
        self.front_right_pub_ = self.create_publisher(ForceSensor, 'hapthexa/leg/front_right/force_sensor_raw', 10)
        self.middle_right_pub_ = self.create_publisher(ForceSensor, 'hapthexa/leg/middle_right/force_sensor_raw', 10)
        self.rear_right_pub_ = self.create_publisher(ForceSensor, 'hapthexa/leg/rear_right/force_sensor_raw', 10)

        self.attitude_pub = self.create_publisher(Attitude, 'hapthexa/attitude', 10)

        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.sensor = serial.Serial()
        self.sensor.baudrate = 115200
        self.sensor.port = "/dev/ttyACM0"
        self.sensor.open()

    def timer_callback(self):
        if self.sensor.in_waiting >= 23:
            if self.sensor.read(1) == b'\xff':
                data = self.sensor.read(22) # センサのデータ21byte+チェックサム1byte
                checksum = 0
                for i in range(21):
                    checksum += data[i]
                if int(checksum) & 0xff == data[21]:
                    msg = ForceSensor()
                    msg.z = data[4] > 127
                    msg.piezo = float(data[4])/255.0
                    msg.loadcell1_raw = data[3]
                    msg.loadcell2_raw = data[5]
                    msg.piezo_raw = data[4]
                    self.front_left_pub_.publish(msg)
                    msg.z = data[7] > 127
                    msg.piezo = float(data[7])/255.0
                    msg.loadcell1_raw = data[6]
                    msg.loadcell2_raw = data[8]
                    msg.piezo_raw = data[7]
                    self.middle_left_pub_.publish(msg)
                    msg.z = data[10] > 127
                    msg.piezo = float(data[10])/255.0
                    msg.loadcell1_raw = data[9]
                    msg.loadcell2_raw = data[11]
                    msg.piezo_raw = data[10]
                    self.rear_left_pub_.publish(msg)
                    msg.z = data[13] > 127
                    msg.piezo = float(data[13])/255.0
                    msg.loadcell1_raw = data[12]
                    msg.loadcell2_raw = data[14]
                    msg.piezo_raw = data[13]
                    self.rear_right_pub_.publish(msg)
                    msg.z = data[16] > 127
                    msg.piezo = float(data[16])/255.0
                    msg.loadcell1_raw = data[15]
                    msg.loadcell2_raw = data[17]
                    msg.piezo_raw = data[16]
                    self.middle_right_pub_.publish(msg)
                    msg.z = data[1] > 127
                    msg.piezo = float(data[1])
                    msg.loadcell1_raw = data[0]
                    msg.loadcell2_raw = data[2]
                    msg.piezo_raw = data[1]
                    self.front_right_pub_.publish(msg)

                    attitude = Attitude()
                    attitude.roll = (data[18] if data[18] < 128 else data[18] - 255)/127*pi
                    attitude.pitch = (data[19] if data[19] < 128 else data[19] - 255)/127*pi
                    attitude.yaw = (data[20] if data[20] < 128 else data[20] - 255)/127*pi
                    self.attitude_pub.publish(attitude)
                    # self.get_logger().info('  vaild checksum {0} == {1}'.format(int(checksum) & 0xff, data[21]))
                else:
                    # self.get_logger().warn('invaild checksum {0} != {1}'.format(int(checksum) & 0xff, data[21]))
                    while self.sensor.in_waiting == 0:
                        time.sleep(0.0001)
                    self.sensor.read(1)
                # self.get_logger().info('data[18-20]: {0},{1},{2}'.format(data[18],data[19],data[20]))


def main(args=None):
    rclpy.init(args=args)

    forcesensor_read = ForcesensorRead()

    rclpy.spin(forcesensor_read)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    forcesensor_read.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()