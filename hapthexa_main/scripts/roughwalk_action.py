#!/usr/bin/env python3

# 参考:
# https://github.com/ros2/examples/tree/master/rclpy/actions

from action_msgs.msg import GoalStatus

from hapthexa_msgs.action import MoveLeg
from hapthexa_msgs.msg import ForceSensor
from hapthexa_msgs.msg import Empty

from hapthexa_msgs.action import Gait

import math
import threading
import time
import sys

import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

# phase = 0


# def forcesensor_callback(msg, node):
#     # node.get_logger().info('{0}'.format(msg.z))
#     if phase == 2 and msg.radial_magnitude > 0.3:
#         node.get_logger().info('z detected')
#         future = goal_handle.cancel_goal_async()
#         future.add_done_callback(lambda future: cancel_done(node, future))
#         global once_failed 
#         once_failed = True

# def cancel_done(node, future):
#     cancel_response = future.result()
#     if len(cancel_response.goals_canceling) > 0:
#         node.get_logger().info('Goal successfully canceled')
#     else:
#         node.get_logger().info('Goal failed to cancel')

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
        self._calibrate_signal_pub = self.create_publisher(Empty, 'calibrate_signal', 10)
        self._move_leg_action_client = ActionClient(self, MoveLeg, 'move_leg')
        self._forcesensor_sub = self.create_subscription(ForceSensor, 'force_sensor', self.forcesensor_callback, 10)
        # self._timer = self.create_timer(0.001, lambda: self.get_logger().info('timer'))

        self._leg_stop_condition = (lambda: False)
        self._leg_stoped = False
        self._move_leg_goal_handle = None

        self._forcesensor_msg = None

        self._x_result = 0.0
        self._y_result = 0.0
        self._z_result = 0.0
        self._z_lock = 0.0

        self._x_offset = 0
        self._z_offset = 0.0

        self._timer_start = time.time()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().debug('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().debug('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute a goal."""
        self.get_logger().debug('Executing goal...')

        w = 5.0 if math.isclose(goal_handle.request.w,0.0) else goal_handle.request.w
        h = 5.0 if math.isclose(goal_handle.request.h,0.0) else goal_handle.request.h
        hmax = 12.0 if math.isclose(goal_handle.request.hmax,0.0) else goal_handle.request.hmax
        hmin = -12.0 if math.isclose(goal_handle.request.hmin,0.0) else goal_handle.request.hmin

        phase = goal_handle.request.phase

        self._z_offset = goal_handle.request.z_offset

        self.get_logger().debug('phase={0}'.format(phase))

        
        if goal_handle.request.is_swingleg:
            if   phase == 1:
                self.move_leg_and_wait(-w/2+self._x_offset, 0, h+self._z_lock)
                self._calibrate_signal_pub.publish(Empty())
            elif phase == 2:
                if self.move_leg_and_wait( w/2+self._x_offset, 0, h+self._z_lock, lambda: self._forcesensor_msg.piezo > 0.5 or self._forcesensor_msg.radial_magnitude > 0.2):
                    self.get_logger().warn('Wall detected. retrying...')
                    self.move_leg_and_wait(-w/2+self._x_offset, 0, hmax+self._z_lock)
                    self._calibrate_signal_pub.publish(Empty())
                    if self.move_leg_and_wait( w/2+self._x_offset, 0, hmax+self._z_lock, lambda: self._forcesensor_msg.piezo > 0.5 or self._forcesensor_msg.radial_magnitude > 0.2):
                        self.get_logger().warn('Wall detected twice. stop')
                        rclpy.shutdown()
                        sys.exit()
            elif phase == 3:
                if not self.move_leg_and_wait( w/2+self._x_offset, 0, hmin, lambda: self._forcesensor_msg.piezo > 0.5 or self._forcesensor_msg.radial_magnitude > 0.2):
                    self.get_logger().warn('Cannot ground. retrying...(1)')
                    self.move_leg_and_wait(w/2+self._x_offset, 0, h+self._z_lock)
                    self._x_offset = -3 if self._x_offset == 3 else self._x_offset + 3
                    self.move_leg_and_wait( w/2+self._x_offset, 0, h+self._z_lock) #phase2動作
                    if not self.move_leg_and_wait( w/2+self._x_offset, 0, hmin, lambda: self._forcesensor_msg.piezo > 0.5 or self._forcesensor_msg.radial_magnitude > 0.2):
                        self.get_logger().warn('Cannot ground. retrying...(2)')
                        self.move_leg_and_wait(w/2+self._x_offset, 0, h+self._z_lock)
                        self._x_offset = -3 if self._x_offset == 3 else self._x_offset + 3
                        self.move_leg_and_wait( w/2+self._x_offset, 0, h+self._z_lock) #phase2動作
                        if not self.move_leg_and_wait( w/2+self._x_offset, 0, hmin, lambda: self._forcesensor_msg.piezo > 0.5 or self._forcesensor_msg.radial_magnitude > 0.2):
                            self.get_logger().warn('Cannot ground. stop')
                            rclpy.shutdown()
                            sys.exit()

                self._z_lock = self._z_result
        else:
            self.move_leg_and_wait( w/2+self._x_offset, 0, self._z_lock)
            self.move_leg_and_wait(-w/2+self._x_offset, 0, self._z_lock)
            self.move_leg_and_wait(-w/2+self._x_offset, 0, self._z_lock)

        goal_handle.succeed()
        self.get_logger().debug('Action succeed')

        result = Gait.Result()
        result.x = self._x_result
        result.y = self._y_result
        result.z = self._z_result
        return result

    def move_leg_and_wait(self, x, y, z, stop_condition=(lambda: False)):

        self._move_leg_action_client.wait_for_server()

        goal_msg = MoveLeg.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.z = float(z+self._z_offset)
        goal_msg.relative_mode = True

        send_goal_future = self._move_leg_action_client.send_goal_async(goal_msg)
        # rclpy.spin_until_future_complete(self, send_goal_future)
        while not send_goal_future.done():
            time.sleep(0.01)
            pass
        self._move_leg_goal_handle = send_goal_future.result()

        self._leg_stop_condition = stop_condition
        self._leg_stoped = False
        self._timer_start = time.time()

        get_result_future = self._move_leg_goal_handle.get_result_async()
        # rclpy.spin_until_future_complete(self, get_result_future)
        while not get_result_future.done():
            time.sleep(0.01)
            pass

        result = get_result_future.result().result
        self._x_result = result.x
        self._y_result = result.y
        self._z_result = result.z
        self.get_logger().debug('z={0}'.format(self._z_result))
        self._move_leg_goal_handle = None

        self._leg_stop_condition = (lambda: False)

        return self._leg_stoped

    def forcesensor_callback(self, msg):
        self._forcesensor_msg = msg
        if self._move_leg_goal_handle is not None and not self._leg_stoped and self._leg_stop_condition():
            self.get_logger().debug('{0}'.format(msg.z))
            future = self._move_leg_goal_handle.cancel_goal_async()
            future.add_done_callback(lambda future: self.cancel_done(future))
            self._leg_stoped = True

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')


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
        self.get_logger().debug('{0}'.format(len(self._leg_names)))
        self._send_goal_futures = [Future]*len(self._leg_names)
        self._get_result_futures = [Future]*len(self._leg_names)
        self._executing_action = [False]*len(self._leg_names)

        self._move_leg_action_client = ActionClient(self, MoveLeg, 'leg/front_left/move_leg')

        self._z_offset = 0

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
        msg.z_offset = float(self._z_offset)

        # msg.is_swingleg = is_group1_true
        # self.execute_gait(0, msg)
        # self.execute_gait(2, msg)
        # self.execute_gait(4, msg)

        # msg.is_swingleg = not is_group1_true
        # self.execute_gait(1, msg)
        # self.execute_gait(3, msg)
        # self.execute_gait(5, msg)

        # self.wait_for_gait()

        self.get_logger().debug('group1' if is_group1_true else 'group2')

        if is_group1_true:
            msg.is_swingleg = True
            msg.phase = 1
            self.execute_gait(0, msg)
            self.execute_gait(2, msg)
            self.execute_gait(4, msg)
            self.wait_for_gait()

            msg.is_swingleg = False
            msg.phase = 0
            self.execute_gait(1, msg)
            self.execute_gait(3, msg)
            self.execute_gait(5, msg)
            self.wait_for_gait()
            msg.is_swingleg = True
            msg.phase = 2
            self.execute_gait(0, msg)
            self.execute_gait(2, msg)
            self.execute_gait(4, msg)
            self.wait_for_gait()

            msg.is_swingleg = True
            msg.phase = 3
            self.execute_gait(0, msg)
            self.execute_gait(2, msg)
            self.execute_gait(4, msg)
            self.wait_for_gait()
        
        else:
            msg.is_swingleg = True
            msg.phase = 1
            self.execute_gait(1, msg)
            self.execute_gait(3, msg)
            self.execute_gait(5, msg)
            self.wait_for_gait()

            msg.is_swingleg = False
            msg.phase = 0
            self.execute_gait(0, msg)
            self.execute_gait(2, msg)
            self.execute_gait(4, msg)
            msg.is_swingleg = True
            msg.phase = 2
            self.execute_gait(1, msg)
            self.execute_gait(3, msg)
            self.execute_gait(5, msg)
            self.wait_for_gait()

            msg.is_swingleg = True
            msg.phase = 3
            self.execute_gait(1, msg)
            self.execute_gait(3, msg)
            self.execute_gait(5, msg)
            self.wait_for_gait()

        
        if is_group1_true:
            self._z_offset = -(self._get_result_futures[0].result().result.z+self._get_result_futures[2].result().result.z+self._get_result_futures[4].result().result.z)/3
        else:
            self._z_offset = -(self._get_result_futures[1].result().result.z+self._get_result_futures[3].result().result.z+self._get_result_futures[5].result().result.z)/3
    
    def execute_gait(self, n, msg):
        self._send_goal_futures[n] = self._gait_action_clients[n].send_goal_async(msg)
        self._executing_action[n] = True
    
    def wait_for_gait(self):

        for i in range(len(self._leg_names)):
            if self._executing_action[i]:
                while not self._send_goal_futures[i].done():
                    time.sleep(0.01)

        for i in range(len(self._leg_names)):
            if self._executing_action[i]:
                self._get_result_futures[i] = self._send_goal_futures[i].result().get_result_async()

        for i in range(len(self._leg_names)):
            if self._executing_action[i]:
                while not self._get_result_futures[i].done():
                    time.sleep(0.01)
                self._executing_action[i] = False

def main(args=None):
    rclpy.init(args=args)
    
    leg_names = ['front_left', 'middle_left', 'rear_left', 'rear_right', 'middle_right', 'front_right']

    executor = MultiThreadedExecutor(num_threads=8) # EachLegGaitノード6スレッド+RoughWalkノード1スレッド+1スレッド
    for leg_name in leg_names:
        executor.add_node(EachLegGait(namespace='hapthexa/leg/'+leg_name))
    executor.add_node(RoughWalk())
    executor.spin()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
