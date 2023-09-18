import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import cv2


class Tb3_laser_sub(Node):
    def __init__(self):
        super().__init__("tb3_laser_sub")
        self.qos_profile = QoSProfile(depth=10)
        self.create_subscription(LaserScan, "/scan", self.sub_message, 10)

    def sub_message(self, msg):
        self.get_logger().info("abcdef")
        print("obstacle :", msg.ranges[360])


def main(args=None):
    rclpy.init(args=args)
    node = Tb3_laser_sub()
    try:
        rclpy.spin(node)  # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt!!")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
