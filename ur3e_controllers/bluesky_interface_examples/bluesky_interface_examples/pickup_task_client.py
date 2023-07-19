import sys

from custom_msgs.srv import TaskCmd
import rclpy
from rclpy.node import Node


class TaskClient(Node):

    def __init__(self):
        super().__init__('pickup_task_client')
        self.cli = self.create_client(TaskCmd, 'ur_task_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TaskCmd.Request()

    def send_request(self, task_number, sample, start, end):
        self.req.task_number = task_number
        self.req.sample_name = sample
        self.req.start_stage = start
        self.req.end_stage = end

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    task_client = TaskClient()
    response = task_client.send_request(int(sys.argv[1]), str(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]) )
    task_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    task_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


#Type -> arg1 -> Over arm pickup: 1 , under arm pickup: 2 