#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from hapthexa_msgs.msg import ForceSensor

class ForceSensorCalibrate(Node):

    def __init__(self):
        super().__init__('force_sensor_calibrate')
        self._force_sensor_raw_sub = self.create_subscription(ForceSensor, 'force_sensor_raw', self.force_sensor_raw_sub_callback,10)
        self.get_logger().info('Hapthexa ForceSensor Calibration')

        self._phase = 0
        self._is_changed = True

        self._loadcell1_zero = 0
        self._loadcell2_zero = 0
        self._piezo_zero = 0

        self._loadcell1_max = 0
        self._loadcell2_max = 0

        self._loadcell1_min = 0
        self._loadcell2_min = 0

        self._loadcell1_flag = 0
        self._loadcell2_flag = 0

        self.declare_parameter("loadcell1_zero", 0)
        self.declare_parameter("loadcell2_zero", 0)
        self.declare_parameter("piezo_zero", 0)
        self.declare_parameter("loadcell1_max", 0)
        self.declare_parameter("loadcell2_max", 0)
        self.declare_parameter("loadcell1_min", 0)
        self.declare_parameter("loadcell2_min", 0)
        self.declare_parameter("loadcell1_flag", 0)
        self.declare_parameter("loadcell2_flag", 0)

        self._count = 0

    def force_sensor_raw_sub_callback(self, msg):
        if self._phase == 0:
            if self._is_changed:
                self.get_logger().info('ゼロ点検出...完了')
                self._loadcell1_zero = msg.loadcell1_raw
                self._loadcell2_zero = msg.loadcell2_raw
                self._piezo_zero = msg.piezo_raw
                self.get_logger().info('対象の脚をロボットの外方向へ引っ張ってください...')
                self._is_changed = False
            else:
                if abs(msg.loadcell1_raw-self._loadcell1_zero) > 20 and abs(msg.loadcell2_raw-self._loadcell2_zero) > 20:
                    self._loadcell1_flag = 1 if msg.loadcell1_raw-self._loadcell1_zero > 0 else -1
                    self._loadcell2_flag = 1 if msg.loadcell2_raw-self._loadcell2_zero > 0 else -1
                    self.get_logger().info('ロードセル方向検出...完了')
                    self._phase += 1
                    self._is_changed = True
        elif self._phase == 1:
            if self._is_changed:
                self.get_logger().info('5秒間の間，脚を全方向に動かしてください...')
                self._count = 0
                self._is_changed = False
            else:
                loadcell1 = self._loadcell1_flag * (msg.loadcell1_raw - self._loadcell1_zero)
                loadcell2 = self._loadcell2_flag * (msg.loadcell2_raw - self._loadcell2_zero)
                if loadcell1 > self._loadcell1_max:
                    self._loadcell1_max = loadcell1
                if loadcell1 < self._loadcell1_min:
                    self._loadcell1_min = loadcell1
                if loadcell2 > self._loadcell2_max:
                    self._loadcell2_max = loadcell2
                if loadcell2 < self._loadcell2_min:
                    self._loadcell2_min = loadcell2

                # self.get_logger().info('self._loadcell2_max: {0},{1}'.format(self._loadcell2_max, loadcell2))
                
                if self._count > 500:
                    self.get_logger().info('ロードセルの最大値最小値取得...完了')
                    self._phase += 1
                    self._is_changed = True
        elif self._phase == 2:
            if self._is_changed:
                self.get_logger().info('キャリブレーション完了')
                self.get_logger().info('ゼロ点: {0}, {1}, {2}'.format(self._loadcell1_zero, self._loadcell2_zero, self._piezo_zero))
                self.get_logger().info('最大値: {0}, {1}'.format(self._loadcell1_max, self._loadcell2_max))
                self.get_logger().info('最小値: {0}, {1}'.format(self._loadcell1_min, self._loadcell2_min))
                self.set_parameters([rclpy.parameter.Parameter("loadcell1_zero", rclpy.Parameter.Type.INTEGER, self._loadcell1_zero)])
                self.set_parameters([rclpy.parameter.Parameter("loadcell1_max", rclpy.Parameter.Type.INTEGER, self._loadcell1_max)])
                self.set_parameters([rclpy.parameter.Parameter("loadcell1_min", rclpy.Parameter.Type.INTEGER, self._loadcell1_min)])
                self.set_parameters([rclpy.parameter.Parameter("loadcell1_flag", rclpy.Parameter.Type.INTEGER, self._loadcell1_flag)])
                self.set_parameters([rclpy.parameter.Parameter("loadcell2_zero", rclpy.Parameter.Type.INTEGER, self._loadcell2_zero)])
                self.set_parameters([rclpy.parameter.Parameter("loadcell2_max", rclpy.Parameter.Type.INTEGER, self._loadcell2_max)])
                self.set_parameters([rclpy.parameter.Parameter("loadcell2_min", rclpy.Parameter.Type.INTEGER, self._loadcell2_min)])
                self.set_parameters([rclpy.parameter.Parameter("loadcell2_flag", rclpy.Parameter.Type.INTEGER, self._loadcell2_flag)])
                self._is_changed = False
                leg_name = self.get_namespace().split('/')[3]
                self.get_logger().info("echo -e 'hapthexa:\n  leg:\n    {0}:' > config/{0}.yaml && ros2 param dump --print /hapthexa/leg/{0}/force_sensor_calibrate | sed -e 's/force_sensor_calibrate/force_sensor/g' | sed -e 's/^/      /g' >> config/{0}.yaml".format(leg_name))

        self._count += 1
        

def main(args=None):
    rclpy.init(args=args)
    force_sensor = ForceSensorCalibrate()
    rclpy.spin(force_sensor)
    force_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()