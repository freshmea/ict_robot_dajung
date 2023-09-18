import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.clock import ClockType, Clock
from std_msgs.msg import Header


class T_pub(Node):
    def __init__(self):
        super().__init__("simple_tpub")
        self.qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(Header, "time", self.qos_profile)
        self.timer = self.create_timer(0.1, self.time_pub)
        self.clock = Clock(clock_type=ClockType.ROS_TIME)
        print(self.clock.clock_type)

    def time_pub(self):
        msg = Header()
        msg.stamp = self.clock.now().to_msg()
        msg.frame_id = "this is time"
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = T_pub()
    try:
        rclpy.spin(node)  # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt!!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
