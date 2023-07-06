import sys
import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient
from my_interface.action import ArithmethicChecker

class Checker(Node):
    def __init__(self):
        super().__init__('checker')
        self.cli = ActionClient(self, ArithmethicChecker, 'arithmetic_checker')

    def send_goal(self, goal_sum):
        goal_msg = ArithmethicChecker.Goal()
        goal_msg.goal_sum = float(goal_sum)
        print('server waingting')
        
        while not self.cli.wait_for_server(timeout_sec=0.5):
            self.get_logger().info('action is not available')
        
        self.send_goal_future = self.cli.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Gaol rejected!!!')
            return
        self.get_logger().info('Goal Acceted!!')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'result: {result.all_formula} {result.total_sum}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.foumula}')

def main(args = None):
    rclpy.init(args=args)
    node = Checker()
    node.send_goal(sys.argv[1])
    rclpy.spin(node)

if __name__ == '__main__':
    main()