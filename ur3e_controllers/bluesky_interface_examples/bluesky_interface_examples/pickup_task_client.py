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

    task_number = 1 # 1 for over arm , 2 for under arm
    sample_name = "sample1"   # this matches the sample you wanna grab ( locations are in the env_objects.yaml file)
    start_stage = 0 
    stop_stage = 1 

    task_client = TaskClient()
    response = task_client.send_request(task_number, sample_name, start_stage, stop_stage)
    task_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    task_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


#Type -> arg1 -> Over arm pickup: 1 , under arm pickup: 2 