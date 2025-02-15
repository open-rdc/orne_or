#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class RealsenseNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/realsense/camera/color/image_raw", Image, self.RGBImageCallback)
        self.depth_sub = rospy.Subscriber("/realsense/camera/depth/image_rect_raw", Image, self.depthCallback)

    def RGBImageCallback(self, rgb_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_image, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("RGB Image", cv_image)
        cv2.waitKey(3)

    def depthCallback(self, depth_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(depth_image, "32FC1")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Depth Image", cv_image)
        cv2.waitKey(3)

def main():
    rospy.init_node("realsense_node")
    realsense_node = RealsenseNode()
    rospy.spin()

if __name__ == "__main__":
    main()

