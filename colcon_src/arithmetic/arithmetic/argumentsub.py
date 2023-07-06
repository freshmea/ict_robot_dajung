import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile 
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from my_interface.msg import ArithmethicArgument

class Argumentsub(Node):
    def __init__(self):
        super().__init__('argumentsub')
        self.qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                                      reliability=QoSReliabilityPolicy.RELIABLE,
                                      durability=QoSDurabilityPolicy.VOLATILE,
                                      depth = 10)
        self.pub = self.create_subscription(ArithmethicArgument, 'arithmetic_argument', self.sub_message, self.qos_profile)

    def sub_message(self, msg):
        self.get_logger().info(f'Recieved message: {msg.stamp.sec} s, {msg.stamp.nanosec} ns')
        self.get_logger().info(f'Recieved message: {msg.argument_a}')
        self.get_logger().info(f'Recieved message: {msg.argument_b}')


def main(args = None):
    rclpy.init(args=args)
    node = Argumentsub()
    try:
        rclpy.spin(node) # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt!!')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()