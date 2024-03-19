#!/usr/bin/env python3

import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header, String
from yolov5_pytorch_ros.msg import BoundingBoxes

class DepthProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        # Depth image subscriber
        self.depth_sub = rospy.Subscriber('/camera/depth/image', Image, self.depth_callback)
        # Bounding boxes subscriber
        self.bbox_sub = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.bbox_callback)
        # Ball color subscriber
        self.color_sub = rospy.Subscriber('/ball_color', String, self.color_callback)
        
        # Object position publisher
        self.position_pub = rospy.Publisher('/object_position', PointStamped, queue_size=10)
        
        self.latest_depth_image = None
        self.latest_bboxes = None
        self.selected_color = None  # 追加: 選択された色を保持

    def color_callback(self, msg):
        self.selected_color = msg.data
        rospy.loginfo(f"Selected color: {self.selected_color}")

    def bbox_callback(self, data):
        self.latest_bboxes = data.bounding_boxes

    def depth_callback(self, depth_data):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")

        if self.latest_depth_image is not None and self.latest_bboxes is not None and self.selected_color is not None:
            self.process_depth_data()

    def process_depth_data(self):
        for bbox in self.latest_bboxes:
            if bbox.Class == f"ball_{self.selected_color}":  # 選択された色に対応するバウンディングボックスのみ処理
                xmin, ymin, xmax, ymax = bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax
                depth_values = self.latest_depth_image[ymin:ymax, xmin:xmax]
                depth_values = depth_values[np.logical_and(depth_values != 0, ~np.isnan(depth_values))]
                if depth_values.size > 0:
                    average_depth = np.nanmean(depth_values)
                    x_center = (xmin + xmax) / 2
                    y_center = (ymin + ymax) / 2
                    position = PointStamped()
                    position.header = Header(frame_id="camera_depth_frame", stamp=rospy.Time.now())
                    position.point.x = x_center
                    position.point.y = y_center
                    position.point.z = average_depth
                    self.position_pub.publish(position)
                    rospy.loginfo(f"Object Class: {bbox.Class}, Position: {position.point}")

if __name__ == '__main__':
    rospy.init_node('depth_processor')
    dp = DepthProcessor()
    rospy.spin()
