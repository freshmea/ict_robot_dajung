import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor 
from my_interface.msg import ArithmethicArgument
from my_interface.srv import ArithmeticOperator
from rclpy.qos import QoSProfile 
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

class Calculator(Node):
    def __init__(self):
        super().__init__('calculator')
        self.argument_a = 0.0
        self.argument_b = 0.0
        self.argument_operator = 0
        self.argumet_result = 0
        self.qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                                      reliability=QoSReliabilityPolicy.RELIABLE,
                                      durability=QoSDurabilityPolicy.VOLATILE,
                                      depth = 10)
        self.argument_subscriber = self.create_subscription(ArithmethicArgument, 'arithmetic_argument', self.get_arithmetic_argument)

        self.arithmethic_service_server = self.create_service(ArithmeticOperator, 'arithmetic_operator', self.get_arthmetic_operator, qos_profile=self.qos_profile, callback_group=ReentrantCallbackGroup())
        
    def get_arithmetic_argument(self, msg):
        self.argument_a = msg.argument_a
        self.argument_b = msg.argument_b
        self.get_logger().info(f'Recieved message: {msg.stamp.sec} s, {msg.stamp.nanosec} ns')
        self.get_logger().info(f'Recieved message: {msg.argument_a}')
        self.get_logger().info(f'Recieved message: {msg.argument_b}')
        
    def get_arthmetic_operator(self, request, response):
        print('get service message')
        self.argument_operator = request.arithmetic_operator
        if self.argument_operator == ArithmeticOperator.Request.PLUS:
            self.argumet_result = self.argument_a + self.argument_b
        elif self.argument_operator == ArithmeticOperator.Request.MINUS:
            self.argumet_result = self.argument_a - self.argument_b
        elif self.argument_operator == ArithmeticOperator.Request.MULTIPLY:
            self.argumet_result = self.argument_a * self.argument_b
        elif self.argument_operator == ArithmeticOperator.Request.DIVISION:
            try:
                self.argumet_result = self.argument_a / self.argument_b
            except:
                self.argumet_result = 0
        response.arithmetic_result = self.argumet_result
        self.get_logger().info(f'Service called {self.argument_a} {self.argument_b} {self.argument_operator} {self.argumet_result}')
        return response


def main(arg= None):
    rclpy.init()
    node = Calculator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrunt')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()