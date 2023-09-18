import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from my_interface.msg import Mytopic


class Myinterface_pub(Node):
    def __init__(self):
        super().__init__("myinterfacepub")
        self.qos_profile = QoSProfile(depth=10)
        self.pub = self.create_publisher(Mytopic, "myfloat", self.qos_profile)
        self.timer = self.create_timer(1, self.spin_msg)
        self.count = 0.0

    def spin_msg(self):
        msg = Mytopic()
        msg.stamp = self.get_clock().now().to_msg()
        msg.my_float = self.count
        self.pub.publish(msg)
        self.count += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = Myinterface_pub()
    try:
        rclpy.spin(node)  # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt!!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
