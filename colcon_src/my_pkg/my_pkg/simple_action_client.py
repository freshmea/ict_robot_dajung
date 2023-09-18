import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_interface.action import Fibonacci


class Fibonacci_client(Node):
    def __init__(self):
        super().__init__("fibonacci_client")
        self.cli = ActionClient(self, Fibonacci, "fibonacci")

    def send_goal(self, step):
        goal_msg = Fibonacci.Goal()
        goal_msg.step = int(step)
        print("server waingting")
        self.cli.wait_for_server()
        self.send_goal_future = self.cli.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        print("sending Goal")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Gaol rejected!!!")
            return
        self.get_logger().info("Goal Acceted!!")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
        print("end of goal_response")

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"result: {result.seq}")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.temp_seq}")


def main(args=None):
    rclpy.init(args=args)
    node = Fibonacci_client()
    node.send_goal(sys.argv[1])
    rclpy.spin(node)


if __name__ == "__main__":
    main()
