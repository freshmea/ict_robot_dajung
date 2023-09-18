import rclpy
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String


class M_pub(Node):
    def __init__(self):
        super().__init__("simple_mpub")
        self.qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(String, "message", self.qos_profile)
        self.pub2 = self.create_publisher(String, "message2", self.qos_profile)
        self.timer = self.create_timer(1, self.spin_msg)
        self.timer2 = self.create_timer(0.5, self.spin_msg2)
        self.count = 0
        self.count2 = 0
        print(sys.version)
        # test = np.zeros((512,512,3), dtype=np.uint8)
        # cv2.imshow(test)

    def spin_msg(self):
        msg = String()
        msg.data = f"hellow {self.count}"
        self.pub.publish(msg)
        self.count += 1

    def spin_msg2(self):
        msg = String()
        msg.data = f"hellow m2 {self.count2}"
        self.pub2.publish(msg)
        self.count2 += 1


def main(args=None):
    rclpy.init(args=args)
    node = M_pub()
    try:
        rclpy.spin(node)  # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt!!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
