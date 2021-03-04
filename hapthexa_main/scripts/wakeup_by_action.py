#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from hapthexa_msgs.action import MoveLeg
from hapthexa_msgs.action import MoveLegArgs

from math import pi
import numpy as np
from scipy.spatial.transform import Rotation

class RoughWalk(Node):

    def __init__(self):
        super().__init__('rough_walk')
        self._leg_names = ['front_left', 'middle_left', 'rear_left', 'rear_right', 'middle_right', 'front_right']
        self._leg_args  = [pi/6.0, pi/2.0, pi*5.0/6.0, -pi*5.0/6.0, -pi/2.0, -pi/6.0]

        self._phase = 0
        self._move_succeed_leg_count = 0

        self._move_leg_action_clients = []
        for leg_name in self._leg_names:
            self._move_leg_action_clients.append(ActionClient(self, MoveLeg, 'hapthexa/leg/'+leg_name+'/move_leg'))

        self._move_leg_args_action_clients = []
        for leg_name in self._leg_names:
            self._move_leg_args_action_clients.append(ActionClient(self, MoveLegArgs, 'hapthexa/leg/'+leg_name+'/move_leg_args'))

        self._send_goal_future = [0]*6
        self._get_result_future = [0]*6
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
            self._send_goal_future[i].add_done_callback(lambda future, i=i: self.goal_response_callback(future, i))

    def move_leg(self, num, x, y, z):
        msg = MoveLeg.Goal()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self._move_leg_action_clients[num].wait_for_server()
        self._send_goal_future[num] = self._move_leg_action_clients[num].send_goal_async(msg)
        self._send_goal_future[num].add_done_callback(lambda future, num=num: self.goal_response_callback(future, num))

    def goal_response_callback(self, future, num):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self._get_result_future[num] = goal_handle.get_result_async()
        self._get_result_future[num].add_done_callback((lambda future: self.get_result_callback(future, num)))

    def get_result_callback(self, future, num):
        result = future.result().result
        self.get_logger().info('result recieved {0}'.format(num))

        self._move_succeed_leg_count += 1
        if self._move_succeed_leg_count == 6:
            self._move_succeed_leg_count = 0
            self._phase += 1
            if self._phase == 1:
                for i in range(6):
                    coxa_length = 4
                    femur_length = 10
                    self.move_leg(i, 0, coxa_length+femur_length if i in {0,1,2} else -(coxa_length+femur_length), 0)
            if self._phase == 2:
                for i in range(6):
                    coxa_length = 4
                    femur_length = 10
                    self.move_leg(i, 0, coxa_length+femur_length if i in {0,1,2} else -(coxa_length+femur_length), -12.5)
            if self._phase == 3:
                for i in range(6):
                    r = Rotation.from_rotvec([0, 0, self._leg_args[i]])
                    t = np.dot(np.array(r.as_matrix()),np.array([[22-8, 0, 0]]).T) + np.array([[0, 0, -12.5]]).T
                    self.move_leg(i, t[0], t[1], t[2])


    
        

def main(args=None):
    rclpy.init(args=args)
    rough_walk = RoughWalk()
    rclpy.spin(rough_walk)
    rough_walk.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()