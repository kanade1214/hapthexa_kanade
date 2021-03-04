#!/usr/bin/env python3

# 参考:
# https://github.com/ros2/examples/tree/master/rclpy/actions

from action_msgs.msg import GoalStatus

from hapthexa_msgs.action import MoveLeg
from hapthexa_msgs.msg import ForceSensor

from hapthexa_msgs.action import Gait

import math
import threading

import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

class EachLegGait(Node):

    def __init__(self, namespace: str = None):
        super().__init__('each_leg_gait', namespace=namespace)
        self._gait_action_server = ActionServer(
            self,
            Gait,
            'gait',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        self._move_leg_action_client = ActionClient(self, MoveLeg, 'move_leg')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute a goal."""
        self.get_logger().info('Executing goal...')

        w = 5.0 if math.isclose(goal_handle.request.w,0.0) else goal_handle.request.w
        h = 5.0 if math.isclose(goal_handle.request.h,0.0) else goal_handle.request.h
        h_max = 12.0 if math.isclose(goal_handle.request.hmax,0.0) else goal_handle.request.hmax
        h_min = -5.0 if math.isclose(goal_handle.request.hmin,0.0) else goal_handle.request.hmin

        if goal_handle.request.is_swingleg:
            self.move_leg_and_wait(-w/2, 0, h)
            self.move_leg_and_wait( w/2, 0, h)
            self.move_leg_and_wait( w/2, 0, 0)
        else:
            self.move_leg_and_wait( w/2, 0, 0)
            self.move_leg_and_wait(-w/2, 0, 0)
            self.move_leg_and_wait(-w/2, 0, 0)

        goal_handle.succeed()
        self.get_logger().info('Action succeed')
        return Gait.Result()

    def move_leg_and_wait(self, x, y, z):
        self._move_leg_action_client.wait_for_server()

        goal_msg = MoveLeg.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.z = float(z)
        goal_msg.relative_mode = True

        send_goal_future = self._move_leg_action_client.send_goal_async(goal_msg)
        # rclpy.spin_until_future_complete(self, send_goal_future)
        while not send_goal_future.done():
            pass
        move_leg_goal_handle = send_goal_future.result()
        get_result_future = move_leg_goal_handle.get_result_async()
        # rclpy.spin_until_future_complete(self, get_result_future)
        while not get_result_future.done():
            pass

class RoughWalk(Node):
    def __init__(self):
        super().__init__('roughwalk',namespace='hapthexa')

        self._leg_names = ['front_left', 'middle_left', 'rear_left', 'rear_right', 'middle_right', 'front_right']
        self._gait_action_clients = []
        for leg_name in self._leg_names:
            self._gait_action_clients.append(ActionClient(self, Gait, 'leg/'+leg_name+'/gait'))

        self._leg_group1 = []
        self._leg_group2 = []
        for i in range(len(self._leg_names)):
            if i % 2:
                self._leg_group1.append(self._leg_names[i])
            else:
                self._leg_group2.append(self._leg_names[i])
        self.get_logger().info('{0}'.format(len(self._leg_names)))
        self._send_goal_futures = [Future]*len(self._leg_names)
        self._get_result_futures = [Future]*len(self._leg_names)

        self._move_leg_action_client = ActionClient(self, MoveLeg, 'leg/front_left/move_leg')


        self._walk_thread = threading.Thread(target=self.walk_thread)
        self._walk_thread.start()


    def walk_thread(self):
        while rclpy.ok():
            self.walk(True)
            self.walk(False)

    def walk(self, is_group1_true: bool=True):

        for i in range(len(self._leg_names)):
            self._gait_action_clients[i].wait_for_server()

        msg = Gait.Goal()
        msg.is_swingleg = is_group1_true
        self._send_goal_futures[0] = self._gait_action_clients[0].send_goal_async(msg)
        self._send_goal_futures[2] = self._gait_action_clients[2].send_goal_async(msg)
        self._send_goal_futures[4] = self._gait_action_clients[4].send_goal_async(msg)
        msg.is_swingleg = not is_group1_true
        self._send_goal_futures[1] = self._gait_action_clients[1].send_goal_async(msg)
        self._send_goal_futures[3] = self._gait_action_clients[3].send_goal_async(msg)
        self._send_goal_futures[5] = self._gait_action_clients[5].send_goal_async(msg)

        while True:
            done = 0
            for i in range(len(self._leg_names)):
                done += self._send_goal_futures[i].done()
            if done == len(self._leg_names):
                break

        for i in range(len(self._leg_names)):
            self._get_result_futures[i] = self._send_goal_futures[i].result().get_result_async()

        while True:
            done = 0
            for i in range(len(self._leg_names)):
                done += self._get_result_futures[i].done()
            if done == len(self._leg_names):
                break

def main(args=None):
    rclpy.init(args=args)
    
    leg_names = ['front_left', 'middle_left', 'rear_left', 'rear_right', 'middle_right', 'front_right']

    executor = MultiThreadedExecutor(num_threads=7) # EachLegGaitノード6スレッド+RoughWalkノード1スレッド+1スレッド
    for leg_name in leg_names:
        executor.add_node(EachLegGait(namespace='hapthexa/leg/'+leg_name))
    executor.add_node(RoughWalk())
    executor.spin()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
