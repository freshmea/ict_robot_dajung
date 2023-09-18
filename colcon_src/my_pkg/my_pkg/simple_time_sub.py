import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Header


class T_sub(Node):
    def __init__(self):
        super().__init__("simple_tsub")
        self.qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sub = self.create_subscription(
            Header, "time", self.sub_time, self.qos_profile
        )

    def sub_time(self, msg):
        self.get_logger().info(f"Recieved time: {msg.stamp.sec}:{msg.stamp.nanosec}")
        self.get_logger().info(f"Recieved frame_id: {msg.frame_id}")


def main(args=None):
    rclpy.init(args=args)
    node = T_sub()
    try:
        rclpy.spin(node)  # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt!!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
