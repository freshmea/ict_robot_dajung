import rclpy
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from my_interface.msg import ArithmethicArgument
from my_interface.srv import ArithmeticOperator
from my_interface.action import ArithmethicChecker
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer


class Calculator(Node):
    def __init__(self):
        super().__init__("calculator")
        self.argument_a = 0.0
        self.argument_b = 0.0
        self.argument_operator = 0
        self.argument_result = 0
        self.argument_formula = ""
        self.argument_symbol = ["+", "-", "*", "/"]
        self.qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.argument_subscriber = self.create_subscription(
            ArithmethicArgument,
            "arithmetic_argument",
            self.get_arithmetic_argument,
            self.qos_profile,
        )
        self.arithmethic_service_server = self.create_service(
            ArithmeticOperator,
            "arithmetic_operator",
            self.get_arthmetic_operator,
            callback_group=ReentrantCallbackGroup(),
        )
        self.arithmetic_action_server = ActionServer(
            self,
            ArithmethicChecker,
            "arithmetic_checker",
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def get_arithmetic_argument(self, msg):
        self.argument_a = msg.argument_a
        self.argument_b = msg.argument_b
        self.get_logger().info(
            f"Recieved message: {msg.stamp.sec} s, {msg.stamp.nanosec} ns"
        )
        self.get_logger().info(f"Recieved message: {msg.argument_a}")
        self.get_logger().info(f"Recieved message: {msg.argument_b}")
        self.argument_formula = f"{self.argument_a} {self.argument_symbol[self.argument_operator-1]} {self.argument_b} = {self.argument_result}"

    def get_arthmetic_operator(self, request, response):
        print("get service message")
        self.argument_operator = request.arithmetic_operator
        if self.argument_operator == ArithmeticOperator.Request.PLUS:
            self.argument_result = self.argument_a + self.argument_b
        elif self.argument_operator == ArithmeticOperator.Request.MINUS:
            self.argument_result = self.argument_a - self.argument_b
        elif self.argument_operator == ArithmeticOperator.Request.MULTIPLY:
            self.argument_result = self.argument_a * self.argument_b
        elif self.argument_operator == ArithmeticOperator.Request.DIVISION:
            if self.argument_b == 0.0:
                self.argument_result = 0.0
            else:
                self.argument_result = self.argument_a / self.argument_b
        response.arithmetic_result = self.argument_result
        print("execution done.")
        self.argument_formula = f"{self.argument_a} {self.argument_symbol[self.argument_operator-1]} {self.argument_b} = {self.argument_result}"
        self.get_logger().info(
            f"Service called {self.argument_a} {self.argument_symbol[self.argument_operator-1]} {self.argument_b} = {self.argument_result}"
        )
        return response

    def execute_callback(self, goal_handle):
        feedback_msg = ArithmethicChecker.Feedback()
        feedback_msg.foumula = []
        total_sum = 0
        goal_sum = goal_handle.request.goal_sum
        while total_sum < goal_sum:
            total_sum += self.argument_result
            feedback_msg.foumula.append(self.argument_formula)
            goal_handle.publish_feedback(feedback_msg)
            print(total_sum, goal_sum)
            time.sleep(1)
        goal_handle.succeed()
        result = ArithmethicChecker.Result()
        result.all_formula = feedback_msg.foumula
        result.total_sum = total_sum
        return result


def main(arg=None):
    rclpy.init()
    node = Calculator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrunt")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
