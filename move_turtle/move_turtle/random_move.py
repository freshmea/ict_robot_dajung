import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile 
from geometry_msgs.msg import Twist 
import random

class TurtleM_pub(Node):
    def __init__(self):
        super().__init__('turtle_mpub')
        self.qos_profile = QoSProfile(depth= 10)
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', self.qos_profile)
        self.pub2 = self.create_publisher(Twist, 'turtle2/cmd_vel', self.qos_profile)
        self.timer = self.create_timer(0.1, self.twist_msg)
        self.timer2 = self.create_timer(0.1, self.twist_msg2)
        self.update_timer = self.create_timer(0.05, self.update)
        self.speed = 2.0
        self.dir = 1.0
        self.speed2 = 2.0
        self.dir2 = 1.0
        self.direction = 0.0

    def twist_msg(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.dir * 2
        self.pub.publish(msg)

    def twist_msg2(self):
        msg = Twist()
        msg.linear.x = self.speed2
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.direction
        self.pub2.publish(msg)

    def update(self):
        self.speed += 0.1 * self.dir
        if self.speed > 10:
            self.dir = -1.0
        elif self.speed < 0:
            self.dir = 1.0

        self.speed2 += 0.3 * self.dir2
        if self.speed2 > 5:
            self.dir2 = -1.0
            self.direction = 3.0
        elif self.speed2 < 0:
            self.dir2 = 1.0
            self.direction = 0.0


def main(args = None):
    rclpy.init(args=args)
    node = TurtleM_pub()
    try:
        rclpy.spin(node) # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt!!')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()