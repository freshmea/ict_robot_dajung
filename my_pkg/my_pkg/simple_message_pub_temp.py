import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile 
from std_msgs.msg import String 


def spin_msg():
    pub.publish(msg)


def main(args = None):
    global pub, msg
    rclpy.init(args=args)
    node = Node('simple_mpub')
    qos_profile = QoSProfile(depth=10)
    pub = node.create_publisher(String, 'message', qos_profile)

    msg = String()
    msg.data = 'hellow'
    timer = node.create_timer(1, spin_msg)
    rclpy.spin(node) # 블럭함수

    print('this is MPUB code')

if __name__ == '__main__':
    main()