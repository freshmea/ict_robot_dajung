import rclpy
from rclpy.node import Node
from my_interface.srv import Mysrv


class Minimal_service(Node):
    def __init__(self):
        super().__init__("twonumber_server")
        self.srv = self.create_service(Mysrv, "twonumber", self.twonumber_callback)
        self.timer = self.create_timer(1, self.test)

    def twonumber_callback(self, request, response):
        self.get_logger().info(f"incomming request {request.first}, {request.second}")
        response.sum = request.first + request.second
        response.multiply = request.first * request.second
        try:
            response.division = float(request.first / request.second)
        except ZeroDivisionError:
            pass
        return response

    def test(self):
        print("spinning....")


def main(args=None):
    rclpy.init(args=args)
    node = Minimal_service()
    try:
        rclpy.spin(node)  # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt!!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
