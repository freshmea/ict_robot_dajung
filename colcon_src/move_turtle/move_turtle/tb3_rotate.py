import rclpy ,sys
from rclpy.node import Node
from rclpy.qos import QoSProfile 
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time
from math import radians, pi, sin , cos, atan2

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class Tb3_rotate(Node):
    def __init__(self):
        super().__init__('tb3_rotate')
        self.pub = self.create_publisher(Twist, 'cmd_vel', depth = 10)
        self.sub = self.create_subscription(Odometry, 'odom', self.update_position_theta)
        self.timer = self.create_timer(0.1, self.spin_msg)
        self.position_x = 0.0
        self.position_y = 0.0
        self.theta = 0.0
        self.dir = 0.0
        self.speed = 0.0

    def update_position_theta(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        _, __, self.theta = euler_from_quaternion(msg.pose.pose.orientation)
        # print('x',self.position_x, '\ny',self.position_y, '\ntheta',self.theta)
        
    def spin_msg(self):
        msg = Twist()
        msg.angular.z = self.dir
        msg.linear.x = self.speed
        self.pub.publish(msg)
    
    def update_org(self):
        self.ori_x = self.position_x
        self.ori_y = self.position_y
        self.ori_theta = self.theta

    def rotate(self, angle):
        rclpy.spin_once(self)
        msg = Twist()
        self.update_org()
        while rclpy.ok():
            rclpy.spin_once(self)
            self.dir1 = angle - self.theta
            self.dir2 = 2*pi - abs(self.dir1)
            self.dir = min(self.dir1, self.dir2)
            print('dir', self.dir)
            if abs(angle - self.theta) < pi/60:
                self.dir = 0.0
                break
            msg.angular.z = self.dir
            self.pub.publish(msg)

    def go_strait(self, distance):
        rclpy.spin_once(self)
        self.target_x = self.position_x + distance * cos(self.theta)
        self.target_y = self.position_y + distance * sin(self.theta)
        msg = Twist()
        self.update_org()
        while rclpy.ok():
            rclpy.spin_once(self)
            diff = abs(self.target_x-self.position_x)+abs(self.target_y-self.position_y)
            self.dir1 = atan2( -self.position_y, -self.position_x) - self.theta
            self.dir2 = 2*pi - abs(self.dir1)
            self.dir = min(self.dir1, self.dir2)/10
            print('diff', diff)
            if diff > 2:
                self.speed = 0.2
            elif diff > 1:
                self.speed = 0.1
            print('speed', self.speed)
            if  diff < 0.3:
                self.speed = 0.0
                self.dir = 0.0
                break
            msg.angular.z = self.dir
            msg.linear.x = self.speed
            self.pub.publish(msg)

    def go_strait2(self, distance):
        rclpy.spin_once(self)
        msg = Twist()
        self.t_time = time.time()
        velocity = 0.2
        self.calc_time = distance / velocity
        while rclpy.ok():
            rclpy.spin_once(self)
            msg.linear.x = velocity
            if  time.time() - self.t_time > self.calc_time:
                self.speed = 0.0
                self.dir = 0.0
                break
            self.pub.publish(msg)

def main(args = None):
    rclpy.init(args=args)
    node = Tb3_rotate()
    try:
        # a = radians(float(input('input rotation degree:')))
        # node.rotate(a)
        d = float(input('input forward meter:'))
        node.go_strait2(d)
        rclpy.spin(node) # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt!!')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()