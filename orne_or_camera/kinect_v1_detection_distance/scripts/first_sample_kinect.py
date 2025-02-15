#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

#class KinectV1DetectDistance:
class KinectNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect/camera/rgb/image_color", Image, self.RGBImageCallback)
        self.depth_sub = rospy.Subscriber("/kinect/camera/depth/image", Image, self.depthCallback)
        self.blue_object_coordinates = []

    def RGBImageCallback(self, rgb_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_image, "bgr8")
            self.blue_object_coordinates = self.detect_blue_object(cv_image)
        except CvBridgeError as e:
            print(e)

    def depthCallback(self, depth_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(depth_image, "32FC1")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Depth Image", cv_image)
        cv2.waitKey(3)

    def detect_blue_object(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        res = cv2.bitwise_and(image, image, mask=mask)
        cv2.imshow("Blue Object", res)
        cv2.waitKey(3)
        return mask

    def calculate_distance(self, depth_image, coordinates):
        pass


def main():
    rospy.init_node("kinect_node")
    kinect_node = KinectNode()
    rospy.spin()

if __name__ == "__main__":
    main()

