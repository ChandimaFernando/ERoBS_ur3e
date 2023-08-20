import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from custom_msgs.action import PickPlace
# from action_tutorials_interfaces.action import Fibonacci


class PickPlaceActionClient(Node):

    def __init__(self):
        super().__init__('erobs_action_client')
        self._action_client = ActionClient(self, PickPlace, 'erbos_pdf_pick_place_action')

    def send_goal(self, sample_name, target_name, task_name):
        goal_msg = PickPlace.Goal()
        goal_msg.sample_name = sample_name
        goal_msg.target_name = target_name
        goal_msg.task_name = task_name

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = PickPlaceActionClient()

    future = action_client.send_goal('holder_shaft_storage', 'holder_shaft_inbeam', "RETURN_PICK_UP")

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()