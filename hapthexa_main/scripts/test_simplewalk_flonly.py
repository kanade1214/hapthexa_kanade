#!/usr/bin/env python3

from action_msgs.msg import GoalStatus

from hapthexa_msgs.action import MoveLeg
from hapthexa_msgs.msg import ForceSensor

import rclpy
from rclpy.action import ActionClient

phase = 0


def forcesensor_callback(msg, node):
    # node.get_logger().info('{0}'.format(msg.z))
    if phase == 2 and msg.radial_magnitude > 0.3:
        node.get_logger().info('z detected')
        future = goal_handle.cancel_goal_async()
        future.add_done_callback(lambda future: cancel_done(node, future))
        global once_failed 
        once_failed = True

def cancel_done(node, future):
    cancel_response = future.result()
    if len(cancel_response.goals_canceling) > 0:
        node.get_logger().info('Goal successfully canceled')
    else:
        node.get_logger().info('Goal failed to cancel')


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('minimal_action_client')
    leg_name = 'front_left'

    forcesensor_sub = node.create_subscription(ForceSensor, 'hapthexa/leg/'+leg_name+'/force_sensor', lambda msg: forcesensor_callback(msg, node),10)

    action_client = ActionClient(node, MoveLeg, 'hapthexa/leg/'+leg_name+'/move_leg')

    node.get_logger().info('Waiting for action server...')
    action_client.wait_for_server()

    global goal_handle

    global phase
    phase = 1

    global once_failed
    once_failed = False

    goal_msg = MoveLeg.Goal()
    goal_msg.x = float(0)
    goal_msg.y = float(0)
    goal_msg.z = float(10.0)
    goal_msg.relative_mode = True

    send_goal_future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, get_result_future)

    #########################

    phase = 2

    goal_msg = MoveLeg.Goal()
    goal_msg.x = float(5.0)
    goal_msg.y = float(0)
    goal_msg.z = float(10.0)
    goal_msg.relative_mode = True
    send_goal_future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, get_result_future)

    #########################


    if once_failed:
        phase = 3

        goal_msg = MoveLeg.Goal()
        goal_msg.x = float(0.0)
        goal_msg.y = float(0)
        goal_msg.z = float(15.0)
        goal_msg.relative_mode = True
        send_goal_future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, send_goal_future)
        goal_handle = send_goal_future.result()
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, get_result_future)

        #########################

        phase = 4

        goal_msg = MoveLeg.Goal()
        goal_msg.x = float(5.0)
        goal_msg.y = float(0)
        goal_msg.z = float(15.0)
        goal_msg.relative_mode = True
        send_goal_future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, send_goal_future)
        goal_handle = send_goal_future.result()
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, get_result_future)

        #########################

    phase = 5

    goal_msg = MoveLeg.Goal()
    goal_msg.x = float(5.0)
    goal_msg.y = float(0)
    goal_msg.z = float(0.0)
    goal_msg.relative_mode = True
    send_goal_future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, get_result_future)

    action_client.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
