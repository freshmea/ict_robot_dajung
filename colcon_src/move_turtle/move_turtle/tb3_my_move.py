import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile 
from geometry_msgs.msg import Twist 
import random

MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 2.84


class TurtleM_pub(Node):
    def __init__(self):
        super().__init__('turtle_mpub')
        self.qos_profile = QoSProfile(depth= 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', self.qos_profile)
        self.timer = self.create_timer(0.1, self.twist_msg)
        self.update_timer = self.create_timer(0.05, self.update)
        self.speed = 2.0
        self.dir = 1.0
        self.direction = 0.0

    def twist_msg(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.dir * 2
        
        self.check_limit(msg)
        self.pub.publish(msg)

    def update(self):
        self.speed += 0.1 * self.dir
        if self.speed > 2:
            self.dir = -1.0
        elif self.speed < 0:
            self.dir = 1.0

    def check_limit(self, msg):
        if msg.linear.x < -MAX_LIN_VEL:
            msg.linear.x = -MAX_LIN_VEL
        elif msg.linear.x > MAX_LIN_VEL:
            msg.linear.x = MAX_LIN_VEL
        if msg.angular.z < -MAX_ANG_VEL:
            msg.angular.z = -MAX_ANG_VEL
        elif msg.angular.z > MAX_ANG_VEL:
            msg.angular.z = MAX_ANG_VEL
        return msg

def main(args = None):
    rclpy.init(args=args)
    node = TurtleM_pub()
    try:
        rclpy.spin(node) # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt!!')
    finally:
        for _ in range(10):
            node.speed = 0.0
            node.dir = 0.0
            node.twist_msg()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()