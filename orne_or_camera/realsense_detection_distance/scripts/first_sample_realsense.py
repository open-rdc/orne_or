#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class RealsenseNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/realsense/camera/color/image_raw", Image, self.RGBImageCallback)
        self.depth_sub = rospy.Subscriber("/realsense/camera/depth/image_rect_raw", Image, self.depthCallback)
        self.x = None
        self.y = None

    def RGBImageCallback(self, rgb_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_image, "bgr8")
            #self.blue_object_coordinates = self.detect_blue_object(cv_image)
            #_ = self.detect_blue_object(cv_image)
        except CvBridgeError as e:
            print(e)

        self.x, self.y, _, _ = self.detect_blue_object(cv_image)
        #cv2.imshow("Depth Image", cv_image)
        #cv2.waitKey(3)

    def depthCallback(self, depth_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(depth_image, "32FC1")
        except CvBridgeError as e:
            print(e)
        if(self.x != None):
            distance = cv_image[self.y, self.x]
            rospy.loginfo("Distance to ball :{} mm ".format(distance))
        cv2.imshow("Depth Image", cv_image)
        cv2.waitKey(3)

    def detect_blue_object(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print(f"len = {len(contours)}")
        if( len(contours) == 0):
            return None, None, None, None
        # 最大の輪郭を見つける
        largest_contour = max(contours, key=cv2.contourArea)
               
        # バウンディングボックスの座標と寸法を取得
        x, y, w, h = cv2.boundingRect(largest_contour)
        # バウンディングボックスの中心座標を計算
        center_x = x + w // 2
        center_y = y + h // 2
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(image, (center_x, center_y), 5, (0, 255, 0), -1)
        cv2.imshow("Blue Object", image)
        cv2.waitKey(3)
        return center_x, center_y, w, h

    def calculate_distance(self, depth_image, center_x, center_y):
        distance = depth_image[center_y, center_x]

        return distance


def main():
    rospy.init_node("realsense_node")
    realsense_node = RealsenseNode()
    rospy.spin()

if __name__ == "__main__":
    main()

