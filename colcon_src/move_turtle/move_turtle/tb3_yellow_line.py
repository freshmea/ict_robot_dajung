import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os


class Tb3_image_sub(Node):
    def __init__(self):
        super().__init__("tb3_image_sub")
        self.qos_profile = QoSProfile(depth=10)
        self.create_subscription(
            Image, "camera1/image_raw", self.sub_message, self.qos_profile
        )
        self.pub = self.create_publisher(Twist, "cmd_vel", self.qos_profile)
        self.create_timer(0.1, self.twist_pub)
        self.cb = CvBridge()
        print(os.getcwd())
        self.traffic = cv2.imread("traffic_stop.png")
        self.orbF = cv2.ORB_create(nfeatures=800)
        self.bf = cv2.BFMatcher_create(cv2.NORM_HAMMING, crossCheck=True)
        self.center = 0.0
        self.findline = False

    def sub_message(self, msg):
        try:
            current_frame = self.cb.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().info(e)

        hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        lower = (15, 0, 0)
        upper = (30, 255, 255)
        hsv = cv2.inRange(hsv, lower, upper)
        ret, mask = cv2.threshold(hsv, 100, 255, cv2.THRESH_BINARY)
        dst = cv2.bitwise_and(current_frame, current_frame, mask=mask)
        line = dst[590, 120:180].copy()
        lineli = []
        for i, pixel in enumerate(line):
            if int(pixel[1]) + int(pixel[2]) > 300:
                lineli.append(i)
        if not lineli:
            self.findline = False
        else:
            self.findline = True
            maxv = max(lineli)
            minv = min(lineli)
            self.center = (minv + maxv) / 2
            print(maxv, minv, self.center)
        cv2.waitKey(1)  # 이미지 처리 시간 주기.
        cv2.imshow("camera", dst)

    def twist_pub(self):
        msg = Twist()
        if self.findline:
            msg.linear.x = 0.1
            self.dir = (20 - self.center) / 40
            msg.angular.z = self.dir
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.5
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Tb3_image_sub()
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
