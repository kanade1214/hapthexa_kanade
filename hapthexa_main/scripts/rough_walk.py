#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from hapthexa_msgs.action import MoveLeg

from math import pi
import numpy as np
from scipy.spatial.transform import Rotation

import signal


class RoughWalk(Node):

    def __init__(self):
        super().__init__('rough_walk')
        self._leg_names = ['front_left', 'middle_left', 'rear_left', 'rear_right', 'middle_right', 'front_right']
        self._leg_args  = [pi/6.0, pi/2.0, pi*5.0/6.0, -pi*5.0/6.0, -pi/2.0, -pi/6.0]
        self._phase = 0
        self._move_succeed_leg_count = 0
        self._w = 8
        self._h = 12
        self._z_lock = 0.0
        self._z_offset = 0.0

        self._exit = False
        signal.signal(signal.SIGINT, self.sigint_callback)

        self._action_clients = []
        for leg_name in self._leg_names:
            self._action_clients.append(ActionClient(self, MoveLeg, 'hapthexa/leg/'+leg_name+'/move_leg'))

        self._send_goal_future = [0]*6
        self._get_result_future = [0]*6

        self._z_result = [0.0]*6
        self._z_lock = [0.0]*6

        for i in range(6):
            self.generate_trajectory(i, 6 if i%2 else 3)

    def generate_trajectory(self, num, phase):
        stop_by_z = False
        result_z = self._z_result[num] + 12.5
        if num == 5:
            self.get_logger().info('result_z= {0},{1}'.format(phase, result_z))
        r = Rotation.from_rotvec([0, 0, self._leg_args[num]])
        t = np.dot(np.array(r.as_matrix()),np.array([[22-8, 0, 0]]).T) + np.array([[0, 0, -12.5+self._z_offset]]).T
        if   phase == 1:
            self._z_lock[num] = 0.0
            t += np.array([[-self._w/2.0, 0, self._h]]).T
        elif phase == 2:
            t += np.array([[ self._w/2.0, 0, self._h]]).T
        elif phase == 3:
            stop_by_z = True
            t += np.array([[ self._w/2.0, 0, -self._h*2/3]]).T
        elif phase == 4:
            self._z_lock[num] = result_z
            t += np.array([[ self._w/2.0, 0, self._z_lock[num]]]).T
        elif phase == 5:
            t += np.array([[-self._w/2.0, 0, self._z_lock[num]]]).T
        elif phase == 6:
            t += np.array([[-self._w/2.0, 0, self._z_lock[num]]]).T
        self.move_leg(num, t[0], t[1], t[2], stop_by_z)


    def move_leg(self, num, x, y, z, stop_by_z=False):
        msg = MoveLeg.Goal()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.abort_if_forcesensor_z_detect_contact = stop_by_z
        self._action_clients[num].wait_for_server()
        self._send_goal_future[num] = self._action_clients[num].send_goal_async(msg)
        self._send_goal_future[num].add_done_callback(lambda future, num=num: self.goal_response_callback(future, num))

    def goal_response_callback(self, future, num):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self._get_result_future[num] = goal_handle.get_result_async()
        self._get_result_future[num].add_done_callback((lambda future: self.get_result_callback(future, num)))

    def sigint_callback(self, sig, frame):
        self._exit = True

    def get_result_callback(self, future, num):
        result = future.result().result
        self.get_logger().info('result recieved {0}'.format(num))
        self._z_result[num] = result.z
        self._move_succeed_leg_count += 1
        if self._move_succeed_leg_count == 6:
            if self._exit:
                self.get_logger().info('SIGINT')
                rclpy.shutdown()
                exit()
            self._move_succeed_leg_count = 0
            self._phase = 1 if self._phase == 6 else self._phase + 1
            if self._phase in {1, 2, 3}:
                self._z_offset = -((self._z_result[0]+self._z_result[2]+self._z_result[4])/3 - (-12.5))
            else:
                self._z_offset = -((self._z_result[1]+self._z_result[3]+self._z_result[5])/3 - (-12.5))
            for i in range(6):
                self.generate_trajectory(i, self._phase if i%2 else self._phase + 3 if self._phase in {1, 2, 3} else self._phase - 3)



    
        

def main(args=None):
    rclpy.init(args=args)
    rough_walk = RoughWalk()
    rclpy.spin(rough_walk)
    rough_walk.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()