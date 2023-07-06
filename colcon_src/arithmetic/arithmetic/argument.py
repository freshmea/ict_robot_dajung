import rclpy 
import random
from rclpy.node import Node
from rclpy.qos import QoSProfile 
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from my_interface.msg import ArithmethicArgument 

class Argument(Node):
    def __init__(self):
        super().__init__('argument')
        self.declare_parameter('min_random_num', 0)
        self.min_random_num = self.get_parameter('min_random_num').value
        self.declare_parameter('max_random_num', 9)
        self.max_random_num = self.get_parameter('max_random_num').value
        # self.add_on_set_parameters_callback(self.update_parameter)
        
        self.qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                                      reliability=QoSReliabilityPolicy.RELIABLE,
                                      durability=QoSDurabilityPolicy.VOLATILE,
                                      depth = 10)
        self.arithmetic_argument_publisher = self.create_publisher(ArithmethicArgument, 'arithmetic_argument', self.qos_profile)
        self.timer = self.create_timer(1, self.spin_msg)

    def spin_msg(self):
        msg = ArithmethicArgument()
        msg.stamp = self.get_clock().now().to_msg()
        msg.argument_a = float(random.randint(self.min_random_num, self.max_random_num))
        msg.argument_b = float(random.randint(self.min_random_num, self.max_random_num))
        self.arithmetic_argument_publisher.publish(msg)
        
    def update_parameter(self, params):
        for param in params:
            if param.name == 'min_random_num':
                self.min_random_num = param.value
            elif param.name == 'max_random_num':
                self.max_random_num = param.value

def main(args = None):
    rclpy.init(args=args)
    node = Argument()
    try:
        rclpy.spin(node) # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt!!')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()