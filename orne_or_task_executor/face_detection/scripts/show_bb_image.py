#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import numpy as np

class BoundingBoxViewer:
    def __init__(self):
        rospy.init_node('bounding_box_viewer', anonymous=True)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 最新の画像を保存する変数
        self.current_image = None
        
        # サブスクライバーの設定
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.bbox_sub = rospy.Subscriber('/face_detection/face_bboxes', Detection2DArray, self.bbox_callback)

        self.image_pub = rospy.Publisher('/face_detection/face_bboxes_image', Image, queue_size=1)
        
        # 表示ウィンドウの名前
        # self.window_name = 'Image with Bounding Boxes'
        # cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        """
        画像メッセージを受け取った時のコールバック関数
        """
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(e)
    
    def bbox_callback(self, msgs):
        """
        バウンディングボックスのメッセージを受け取った時のコールバック関数
        """
        if self.current_image is None:
            return
            
        # 画像のコピーを作成
        display_image = self.current_image.copy()

        draw_max_width = display_image.shape[1]
        draw_max_height = display_image.shape[0]
        draw_min_width = 0
        draw_min_height = 0


        
        # 各バウンディングボックスを描画
        for detection in msgs.detections:
            bbox = detection.bbox
            center_x = bbox.center.x
            center_y = bbox.center.y
            width = bbox.size_x
            height = bbox.size_y 

            score = detection.results[0].score
            
            # バウンディングボックスを描画
            cv2.rectangle(display_image, (np.clip(center_x-width/2, draw_min_width, draw_max_width), np.clip(center_y-height/2, draw_min_height, draw_max_height)), (np.clip(center_x + width/2, draw_min_width, draw_max_width), np.clip(center_y + height/2, draw_min_height, draw_max_height)), (0, 255, 0), 2)
            
            # 信頼度を描画
            cv2.putText(display_image, f"{score:.2f}", (np.clip(center_x-width/2, draw_min_width, draw_max_width), np.clip(center_y-height/2, draw_min_height, draw_max_height)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 画像を表示
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(display_image, "bgr8"))
        # cv2.imshow(self.window_name, display_image)
        # cv2.waitKey(1)

    def run(self):
        """
        メインループ
        """
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    viewer = BoundingBoxViewer()
    viewer.run()
