import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile 
from std_msgs.msg import String 

class M_pub(Node):
    def __init__(self):
        super().__init__('simple_mpub')
        self.qos_profile = QoSProfile(depth=10)
        self.pub = self.create_publisher(String, 'message', self.qos_profile)
        self.timer = self.create_timer(1, self.spin_msg)
        self.count = 0

    def spin_msg(self):
        msg = String()
        msg.data = f'hellow {self.count}'
        self.pub.publish(msg)
        self.count += 1


def main(args = None):
    rclpy.init(args=args)
    node = M_pub()
    try:
        rclpy.spin(node) # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt!!')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()