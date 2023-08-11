import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from custom_msgs.action import PickPlace
# from action_tutorials_interfaces.action import Fibonacci


class PickPlaceActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, PickPlace, 'PickPlaceAct_task')

    def send_goal(self, order):
        goal_msg = PickPlace.Goal()
        goal_msg.sample_name = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = PickPlaceActionClient()

    future = action_client.send_goal('sample1')

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()