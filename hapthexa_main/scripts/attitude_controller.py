#!/usr/bin/env python3

import rclpy
import rclpy.node
#  import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient

from hapthexa_msgs.msg import LegPosition
from hapthexa_msgs.msg import Attitude
from hapthexa_msgs.action import Empty
from hapthexa_msgs.action import MoveLeg
from hapthexa_msgs.action import MoveLegArgs

from rcl_interfaces.msg import ParameterEvent

from math import pi
import numpy as np
from scipy.spatial.transform import Rotation

class LowPass:
    def __init__(self, a):
        self._a = 1.0 if a > 1.0 else 0.0 if a < 0.0 else a
        self._previous_ret = 0.0

    def lpf(self, value):
        ret = self._a * self._previous_ret + (1.0-self._a) * value
        self._previous_ret = ret  
        return ret

class PID:
    def __init__(self):
        self._kp = 1.0
        self._kd = 0.001
        self._ki = 1.0
        self._t  = 0.01

        self._sum = 0.0
        self._previous_error = 0.0
        self._first_time = True

        self._max =  pi/4
        self._min = -pi/4

        self._lpf = LowPass(0.7)
    
    def pid(self, error):
        self._sum += error * self._t
        lpf_error = self._lpf.lpf(error)
        diff = (lpf_error - self._previous_error) / self._t
        if self._first_time:
            diff = 0.0
            self._first_time = False
        self._previous_error = lpf_error

        ret = self._kp * lpf_error + self._ki * self._sum + self._kd * diff
        return self._max if ret > self._max else self._min if ret < self._min else ret

    def set_gain(self, kp, ki, kd):
        self._kp = kp
        self._kd = kd
        self._ki = ki

    def reset(self):
        self._sum = 0.0
        self._previous_error = 0.0
        self._first_time = True

def rotation_matrix_from_ypr(yaw, pitch, roll):
    r_yaw   = np.array(Rotation.from_rotvec([0   , 0    , yaw]).as_matrix())
    r_pitch = np.array(Rotation.from_rotvec([0   , pitch, 0  ]).as_matrix())
    r_roll  = np.array(Rotation.from_rotvec([roll, 0    , 0  ]).as_matrix())
    return np.dot(r_yaw, np.dot(r_pitch, r_roll))

class AttitudeController(rclpy.node.Node):

    def __init__(self):
        super().__init__('attitude_controller')

        self._enable_attitude_control = False

        self._leg_names = ['front_left', 'middle_left', 'rear_left', 'rear_right', 'middle_right', 'front_right']
        self._leg_args  = [pi/6.0, pi/2.0, pi*5.0/6.0, -pi*5.0/6.0, -pi/2.0, -pi/6.0]
        self._leg_position_pubs = []
        for leg_name in self._leg_names:
            self._leg_position_pubs.append(self.create_publisher(LegPosition, 'hapthexa/leg/'+leg_name+'/leg_position', 10))
        
        self._attitude_sub = self.create_subscription(Attitude, 'hapthexa/attitude', self.attitude_callback,10)

        self._parameter_event_sub = self.create_subscription(ParameterEvent, 'parameter_events', self.parameter_callback,10)
        self.declare_parameter("kp", 0.01)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.0)

        self.declare_parameter("roll_goal", 0.0)
        self.declare_parameter("pitch_goal", 0.0)

        self._wakeup_action_server = ActionServer(
            self,
            Empty,
            'hapthexa/wakeup',
            self.wakeup_action_callback
        )

        self._wakeup_action_server = ActionServer(
            self,
            Empty,
            'hapthexa/default',
            self.default_action_callback
        )

        self._lie_action_server = ActionServer(
            self,
            Empty,
            'hapthexa/lie',
            self.lie_action_callback
        )

        self._move_succeed_leg_count = 0
        self._move_leg_action_clients = []
        for leg_name in self._leg_names:
            self._move_leg_action_clients.append(ActionClient(self, MoveLeg, 'hapthexa/leg/'+leg_name+'/move_leg'))

        self._move_leg_args_action_clients = []
        for leg_name in self._leg_names:
            self._move_leg_args_action_clients.append(ActionClient(self, MoveLegArgs, 'hapthexa/leg/'+leg_name+'/move_leg_args'))

        self._send_goal_future = [0]*6
        self._get_result_future = [0]*6
        # self._get_result_future[1] = rclpy.task.Future()

        timer_period = 0.01
        self._timer = self.create_timer(timer_period, self.timer_callback)


        self.declare_parameter("enable_pid", True)

        self._leg_circle_radius = 22.0
        self._leg_root_radius = 8.0

        self._bias_x = 0.0
        self._bias_y = 0.0
        self._bias_z = 15.0

        self._robot_roll = 0.0
        self._robot_pitch = 0.0 

        self._roll_pid = PID()
        self._pitch_pid = PID()
        
        self._roll_goal = 0.0
        self._pitch_goal = 0.0

    def move_leg(self, num, x, y, z):
        msg = MoveLeg.Goal()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self._move_leg_action_clients[num].wait_for_server()
        self._send_goal_future[num] = self._move_leg_action_clients[num].send_goal_async(msg)
        self._send_goal_future[num].add_done_callback(lambda future, num=num: self.action_goal_response_callback(future, num))

    def move_leg_default_position(self, num):
        msg = MoveLeg.Goal()
        msg.relative_mode = True
        self._move_leg_action_clients[num].wait_for_server()
        self._send_goal_future[num] = self._move_leg_action_clients[num].send_goal_async(msg)
        self._send_goal_future[num].add_done_callback(lambda future, num=num: self.action_goal_response_callback(future, num))

    def wakeup_action_callback(self, goal_handle):
        self.get_logger().info('wakeup')

        self._move_succeed_leg_count = 0
        for i in range(6):
            # r = Rotation.from_rotvec([0, 0, self._leg_args[i]])
            # t = np.dot(np.array(r.as_matrix()),np.array([[22-8, 0, 0]]).T) + np.array([[0, 0, -12.5]]).T

            msg = MoveLegArgs.Goal()
            if i in {0, 3}:
                msg.coxa_arg = pi/2
            elif i in {2, 5}:
                msg.coxa_arg = -pi/2
            else:
                msg.coxa_arg = 0.0
            msg.femur_arg = float(pi/2)
            msg.tibia_arg = float(-pi*5/6)
            self._move_leg_args_action_clients[i].wait_for_server()
            self._send_goal_future[i] = self._move_leg_args_action_clients[i].send_goal_async(msg)
            self._send_goal_future[i].add_done_callback(lambda future, i=i: self.action_goal_response_callback(future, i))
        while rclpy.ok() and self._move_succeed_leg_count != 6:
            rclpy.spin_once(self)

        self._move_succeed_leg_count = 0
        for i in range(6):
            r = Rotation.from_rotvec([0, 0, self._leg_args[i]])
            t = np.dot(np.array(r.as_matrix()),np.array([[22-8, 0, 0]]).T) + np.array([[0, 0, 0]]).T
            self.move_leg(i, t[0], t[1], t[2])
        while rclpy.ok() and self._move_succeed_leg_count != 6:
            rclpy.spin_once(self)

        self._move_succeed_leg_count = 0
        for i in range(6):
            r = Rotation.from_rotvec([0, 0, self._leg_args[i]])
            t = np.dot(np.array(r.as_matrix()),np.array([[22-8, 0, 0]]).T) + np.array([[0, 0, -self._bias_z]]).T
            self.move_leg(i, t[0], t[1], t[2])
        while rclpy.ok() and self._move_succeed_leg_count != 6:
            rclpy.spin_once(self)

        self._pitch_pid.reset()
        self._roll_pid.reset()
        self._enable_attitude_control = True
        self._robot_roll = 0.0
        self._robot_pitch = 0.0 
        
        goal_handle.succeed()
        return Empty.Result()

    def default_action_callback(self, goal_handle):
        self.get_logger().info('default')

        self._move_succeed_leg_count = 0
        for i in range(6):
            self.move_leg_default_position(i)
        while rclpy.ok() and self._move_succeed_leg_count != 6:
            rclpy.spin_once(self)
        
        goal_handle.succeed()
        return Empty.Result()

    def lie_action_callback(self, goal_handle):
        self.get_logger().info('wakeup')

        self._enable_attitude_control = False

        self._move_succeed_leg_count = 0
        for i in range(6):
            r = Rotation.from_rotvec([0, 0, self._leg_args[i]])
            t = np.dot(np.array(r.as_matrix()),np.array([[22-8, 0, 0]]).T) + np.array([[0, 0, -self._bias_z]]).T
            self.move_leg(i, t[0], t[1], t[2])
        while rclpy.ok() and self._move_succeed_leg_count != 6:
            rclpy.spin_once(self)

        self._move_succeed_leg_count = 0
        for i in range(6):
            r = Rotation.from_rotvec([0, 0, self._leg_args[i]])
            t = np.dot(np.array(r.as_matrix()),np.array([[22-8, 0, 0]]).T) + np.array([[0, 0, 0]]).T
            self.move_leg(i, t[0], t[1], t[2])
        while rclpy.ok() and self._move_succeed_leg_count != 6:
            rclpy.spin_once(self)


        self._move_succeed_leg_count = 0
        for i in {1, 3, 5}:
            # r = Rotation.from_rotvec([0, 0, self._leg_args[i]])
            # t = np.dot(np.array(r.as_matrix()),np.array([[22-8, 0, 0]]).T) + np.array([[0, 0, -12.5]]).T

            msg = MoveLegArgs.Goal()
            if i in {0, 3}:
                msg.coxa_arg = pi/2
            elif i in {2, 5}:
                msg.coxa_arg = -pi/2
            else:
                msg.coxa_arg = 0.0
            msg.femur_arg = float(pi/2)
            msg.tibia_arg = float(pi/2 + pi/12)
            self._move_leg_args_action_clients[i].wait_for_server()
            self._send_goal_future[i] = self._move_leg_args_action_clients[i].send_goal_async(msg)
            self._send_goal_future[i].add_done_callback(lambda future, i=i: self.action_goal_response_callback(future, i))
        while rclpy.ok() and self._move_succeed_leg_count != 3:
            rclpy.spin_once(self)

        self._move_succeed_leg_count = 0
        for i in {0, 2, 4}:
            # r = Rotation.from_rotvec([0, 0, self._leg_args[i]])
            # t = np.dot(np.array(r.as_matrix()),np.array([[22-8, 0, 0]]).T) + np.array([[0, 0, -12.5]]).T

            msg = MoveLegArgs.Goal()
            if i in {0, 3}:
                msg.coxa_arg = pi/2
            elif i in {2, 5}:
                msg.coxa_arg = -pi/2
            else:
                msg.coxa_arg = 0.0
            msg.femur_arg = float(pi/2)
            msg.tibia_arg = float(pi/2 - pi/12)
            self._move_leg_args_action_clients[i].wait_for_server()
            self._send_goal_future[i] = self._move_leg_args_action_clients[i].send_goal_async(msg)
            self._send_goal_future[i].add_done_callback(lambda future, i=i: self.action_goal_response_callback(future, i))
        while rclpy.ok() and self._move_succeed_leg_count != 3:
            rclpy.spin_once(self)
        
        goal_handle.succeed()
        return Empty.Result()

    def action_goal_response_callback(self, future, num):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self.get_logger().info('goal accepted {0}'.format(num))
        self._get_result_future[num] = goal_handle.get_result_async()
        self._get_result_future[num].add_done_callback((lambda future: self.action_get_result_callback(future, num)))

    def action_get_result_callback(self, future, num):
        result = future.result().result
        self.get_logger().info('result recieved {0}'.format(num))
        self._move_succeed_leg_count += 1

    def parameter_callback(self, msg):
        self.get_logger().info('parameter_callback called')
        kp = self.get_parameter("kp").get_parameter_value().double_value
        ki = self.get_parameter("ki").get_parameter_value().double_value
        kd = self.get_parameter("kd").get_parameter_value().double_value
        self._roll_goal = self.get_parameter("roll_goal").get_parameter_value().double_value
        self._pitch_goal = self.get_parameter("pitch_goal").get_parameter_value().double_value
        self.get_logger().info('kp={0}, ki={1}, kd={2}'.format(kp, ki, kd))
        self._roll_pid.set_gain(kp, ki, kd)
        self._pitch_pid.set_gain(kp, ki, kd)
        self._roll_pid.reset()
        self._pitch_pid.reset()

    def attitude_callback(self, msg):
        if self.get_parameter("enable_pid").get_parameter_value().bool_value:
            self._robot_pitch += self._pitch_pid.pid(msg.pitch-self._pitch_goal)
            self._robot_roll += self._roll_pid.pid(msg.roll-self._roll_goal)
            if self._robot_pitch > pi/8:
                self._robot_pitch = pi/8
            elif self._robot_pitch < -pi/8:
                self._robot_pitch = -pi/8
            if self._robot_roll > pi/8:
                self._robot_roll = pi/8
            elif self._robot_roll < -pi/8:
                self._robot_roll = -pi/8

            self._bias_x = np.clip(self._robot_pitch/(pi/6)*5, -5, 5)
            self._bias_y = np.clip(-self._robot_roll/(pi/6)*5, -5, 5)

        else:
            self._robot_pitch = 0.0
            self._robot_roll = 0.0

    def timer_callback(self):
        if not self._enable_attitude_control:
            return
        for i in range(6):
            # rot = np.array(Rotation.from_rotvec([0, 0, self._leg_args[i]]).as_matrix())
            cir = np.dot(rotation_matrix_from_ypr(self._leg_args[i], 0, 0),np.array([[self._leg_circle_radius, 0, 0]]).T)
            rot = rotation_matrix_from_ypr(0, self._robot_pitch, self._robot_roll)
            bias = np.array([[-self._bias_x, -self._bias_y, -self._bias_z]]).T
            root = np.dot(rotation_matrix_from_ypr(self._leg_args[i], 0, 0),np.array([[self._leg_root_radius, 0, 0]]).T)
            t = np.dot(rot, cir) + bias - root
            msg = LegPosition()
            msg.x = float(t[0])
            msg.y = float(t[1])
            msg.z = float(t[2])
            self._leg_position_pubs[i].publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = AttitudeController()
    # rclpy.spin(node)
    while rclpy.ok():
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()