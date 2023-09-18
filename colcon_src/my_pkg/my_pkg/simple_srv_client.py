import sys
import rclpy
from rclpy.node import Node
from my_interface.srv import Mysrv


class Minimal_client(Node):
    def __init__(self):
        super().__init__("twonumber_client")
        self.cli = self.create_client(Mysrv, "twonumber")
        while not self.cli.wait_for_service(2.0):
            self.get_logger().info("waiting...")
        self.req = Mysrv.Request()

    def send_request(self, first, second):
        self.req.first = int(first)
        self.req.second = int(second)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    node = Minimal_client()
    response = node.send_request(sys.argv[1], sys.argv[2])
    node.get_logger().info(
        f"sum: {response.sum}, multiply: {response.multiply},\
        division: {response.division}"
    )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
