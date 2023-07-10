import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile 
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
import random
from math import atan2, sqrt, pi
import numpy as np 
import sys, os

MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 2.84

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

class TurtleM_pub(Node):
    def __init__(self):
        super().__init__('turtle_mpub')
        self.qos_profile = QoSProfile(depth= 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', self.qos_profile)
        self.sub = self.create_subscription(Odometry, 'odom', self.update_position_theta)
        self.timer = self.create_timer(0.1, self.twist_msg)
        self.update_timer = self.create_timer(0.05, self.update)
        self.speed = 2.0
        self.dir = 1.0
        self.direction = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.theta = 0.0
        self.t_theta = 0.0
        self.moving_time = 0.0
        self.target_theta = 0.0
        self.t_target_theta = 0.0
        self.distance = 100.0
        

    def twist_msg(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.dir
        
        self.check_limit(msg)
        self.pub.publish(msg)
        
    def update_position_theta(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        _, __, self.theta = euler_from_quaternion(msg.pose.pose.orientation)
        print('x',self.position_x, '\ny',self.position_y, '\ntheta',self.theta)
    
    def calc_theta(self):
        os.system('clear')
        self.target_theta = atan2(-self.position_y, -self.position_x)
        print('target_theta',self.target_theta)
        self.distance = sqrt(self.position_x**2 + self.position_y**2)
        print('\ndis:', self.distance)
        print('\nself.dir', self.dir)
        print('\nself.speed', self.speed)
        # self.moving_time = self.distance/self.speed
        
    def update(self):
        self.dir1 = (self.target_theta - self.theta)
        self.dir2 = 2*pi - abs(self.dir1)
        self.dir = min(self.dir1, self.dir2)
        if self.distance < 0.1:
            self.speed = 0.0
            self.dir = 0.0
        elif self.distance < 1:
            self.speed = 1.0
        else:
            self.speed = 2.0
        self.calc_theta()

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