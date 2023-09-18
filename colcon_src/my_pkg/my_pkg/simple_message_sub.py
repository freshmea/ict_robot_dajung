import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String


class M_sub(Node):
    def __init__(self):
        super().__init__("simple_msub")
        self.qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_subscription(
            String, "message", self.sub_message, self.qos_profile
        )

    def sub_message(self, msg):
        self.get_logger().info(f"Recieved message: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = M_sub()
    try:
        rclpy.spin(node)  # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt!!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
