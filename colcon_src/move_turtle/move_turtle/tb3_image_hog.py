import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, LaserScan
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import os, sys
from pathlib import Path


class Tb3_image_sub(Node):
    def __init__(self):
        super().__init__('tb3_image_sub')
        self.qos_profile = QoSProfile(depth = 10)
        self.create_subscription(Image, 'camera1/image_raw', self.sub_message, 10)
        self.create_subscription(LaserScan, '/scan', self.sub_scan, 10)
        self.cb = CvBridge()
        print(os.getcwd())
        self.traffic = cv2.imread('traffic_stop.png')
        self.hog = cv2.HOGDescriptor(
            _winSize=(64,128),
            _blockSize=(16,16),
            _blockStride=(8,8),
            _cellSize=(8,8),
            _nbins=9,
            _derivAperture=1,
            _winSigma= -1,
            _histogramNormType = 0,
            _L2HysThreshold=0.2,
            _gammaCorrection=True,
            _nlevels=64,
            _signedGradient=False
            )
        # self.hog = cv2.HOGDescriptor()
        self.des = self.hog.compute(self.traffic)

    def sub_message(self, msg):
        try:
            current_frame = self.cb.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().info(e)


        loc1, weights1 = self.hog.detectMultiScale(current_frame)
        
        print(loc1)
        w, h =self.hog.winSize
        for pt in loc1:
            x,y = pt 
            cv2.rectangle(current_frame, (x,y), (x+w,y+h), (0,255,0), 2)
            
        print('center pixel', current_frame[300,300])
        cv2.waitKey(1) # 이미지 처리 시간 주기.
        cv2.imshow('camera', current_frame)
        
    def sub_scan(self, msg):
        self.get_logger().info('abcdef')
        print('obstacle :', msg.ranges[360])


def main(args = None):
    rclpy.init(args=args)
    node = Tb3_image_sub()
    try:
        rclpy.spin(node) # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt!!')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()