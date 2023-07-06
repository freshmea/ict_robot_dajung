import sys
import random
import rclpy 
from rclpy.node import Node
from my_interface.srv import ArithmeticOperator

class Operator(Node):
    def __init__(self):
        super().__init__('operator')
        self.cli = self.create_client(ArithmeticOperator, 'arithmetic_operator')
        while not self.cli.wait_for_service(2.0):
            self.get_logger().info('waiting...')
        self.req = ArithmeticOperator.Request()

    def send_request(self):
        self.req.arithmetic_operator = random.randint(1, 4)
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args = None):
    rclpy.init(args=args)
    node = Operator()
    future = node.send_request()
    user_trigger = True
    while rclpy.ok():
        if user_trigger:
            rclpy.spin_once(node)
            if future.done():
                try:
                    service_response = future.result()
                except:
                    node.get_logger().info('service calling fail!!')
                else:
                    node.get_logger().info(f'{service_response.arithmetic_result}')
                    user_trigger = False
        else:
            input('Press Enter for next service call')
            future = node.send_request()
            user_trigger = True

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()