#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from yolov5_pytorch_ros.msg import BoundingBoxes

class DepthProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/camera/depth/image', Image, self.depth_callback)
        self.bbox_sub = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.bbox_callback)
        
        self.depth_pub = rospy.Publisher('/object_average_depth', Float32, queue_size=10)
        
        self.latest_depth_image = None
        self.latest_bboxes = None

    def bbox_callback(self, data):
        self.latest_bboxes = data.bounding_boxes

    def depth_callback(self, depth_data):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")

        if self.latest_depth_image is not None and self.latest_bboxes is not None:
            self.process_depth_data()

    def process_depth_data(self):
        for bbox in self.latest_bboxes:
            # 対象のオブジェクトクラスを['ball_red', 'ball_blue', 'ball_yellow']に更新
            if bbox.Class in ['ball_red', 'ball_blue', 'ball_yellow']:
                xmin, ymin, xmax, ymax = bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax
                depth_values = self.latest_depth_image[ymin:ymax, xmin:xmax]
                depth_values = depth_values[np.logical_and(depth_values != 0, ~np.isnan(depth_values))]
                if depth_values.size > 0:
                    average_depth = np.nanmean(depth_values)
                    self.depth_pub.publish(Float32(average_depth))
                    rospy.loginfo(f"Object Class: {bbox.Class}, Average Depth: {average_depth}")
                else:
                    rospy.loginfo(f"Object Class: {bbox.Class}, Depth data not available or invalid.")

if __name__ == '__main__':
    rospy.init_node('depth_processor')
    dp = DepthProcessor()
    rospy.spin()