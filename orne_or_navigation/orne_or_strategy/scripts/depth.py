#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from yolov5_pytorch_ros.msg import BoundingBoxes

class DepthProcessor:
    def __init__(self):
        rospy.init_node('depth_processor', anonymous=True)
        
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/camera/depth/image', Image, self.depth_callback)
        self.bbox_sub = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.bbox_callback)
        self.color_sub = rospy.Subscriber('/ball_color', String, self.color_callback)
        
        self.depth_pub = rospy.Publisher('/object_average_depth', Float32, queue_size=10)
        
        self.latest_depth_image = None
        self.target_color = None
        self.color_received = False

    def color_callback(self, data):
        self.target_color = data.data
        self.color_received = True

    def bbox_callback(self, data):
        self.latest_bboxes = data.bounding_boxes

    def depth_callback(self, depth_data):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")

        if self.latest_depth_image is not None:
            self.process_depth_data()

    def process_depth_data(self):
        if not self.color_received:
            # If color has not been received, calculate and publish average depth for each bounding box
            for bbox in self.latest_bboxes:
                self.calculate_and_publish_depth(bbox)
        else:
            # Once color is received, calculate depth for the target color object only
            for bbox in self.latest_bboxes:
                if self.target_color in bbox.Class:
                    self.calculate_and_publish_depth(bbox)
                    break  # Assuming only one target object is needed

    def calculate_and_publish_depth(self, bbox):
        depth_values = self.extract_depth_values(bbox)
        if depth_values.size > 0:
            average_depth = np.nanmean(depth_values)
            self.depth_pub.publish(average_depth)
            rospy.loginfo(f"Object Class: {bbox.Class}, Average Depth: {average_depth}")

    def extract_depth_values(self, bbox):
        xmin, ymin, xmax, ymax = bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax
        depth_values = self.latest_depth_image[ymin:ymax, xmin:xmax]
        return depth_values[np.logical_not(np.isnan(depth_values))]

if __name__ == '__main__':
    DepthProcessor()
    rospy.spin()
