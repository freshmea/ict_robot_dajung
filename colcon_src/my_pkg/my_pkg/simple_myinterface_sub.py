import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from my_interface.msg import Mytopic


class Myinterface_sub(Node):
    def __init__(self):
        super().__init__("myinterfacesub")
        self.qos_profile = QoSProfile(depth=10)
        self.sub = self.create_subscription(
            Mytopic, "myfloat", self.sub_time, self.qos_profile
        )

    def sub_time(self, msg):
        self.get_logger().info(f"Recieved time: {msg.stamp.sec}:{msg.stamp.nanosec}")
        self.get_logger().info(f"Recieved my_float: {msg.my_float}")


def main(args=None):
    rclpy.init(args=args)
    node = Myinterface_sub()
    try:
        rclpy.spin(node)  # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt!!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
