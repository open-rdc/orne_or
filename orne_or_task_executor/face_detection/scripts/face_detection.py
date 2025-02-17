#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

import insightface
from insightface.app import FaceAnalysis
from insightface.data import get_image as ins_get_image


def get_bbox(bbox):
    center_x = bbox[0] + bbox[2] / 2
    center_y = bbox[1] + bbox[3] / 2
    size_x = bbox[2] - bbox[0]
    size_y = bbox[3] - bbox[1]
    return center_x, center_y, size_x, size_y

class FaceDetectionNode:
    def __init__(self):
        self.face_analysis = FaceAnalysis(providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
        self.face_analysis.prepare(ctx_id=0, det_size=(640, 480))

        # ノードの初期化
        rospy.init_node('facedetectionnode', anonymous=True)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/face_detection/face_bboxes', Detection2DArray, queue_size=1)
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        
        # ループのレート設定
        self.rate = rospy.Rate(10)  # 10Hz

    def image_callback(self, msg):
        try:
            # ROSの画像メッセージをOpenCVの画像に変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 顔検出
            self.face_detection(cv_image)
            
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def face_detection(self, image):
        # 顔検出
        detection2d_array_pub = Detection2DArray()
        headear = Header()
        headear.stamp = rospy.Time.now()

        faces = self.face_analysis.get(image)
        for face in faces:
            detection2d = Detection2D()
            detection2d.header = headear
            center_x, center_y, size_x, size_y = get_bbox(face.bbox)
            detection2d.bbox.center.x = center_x
            detection2d.bbox.center.y = center_y
            detection2d.bbox.size_x = size_x
            detection2d.bbox.size_y = size_y

            results = ObjectHypothesisWithPose()
            results.id = 0
            results.score = face.det_score
            detection2d.results.append(results)
            detection2d_array_pub.detections.append(detection2d)

        detection2d_array_pub.header = headear
        self.image_pub.publish(detection2d_array_pub)


    """
    def get_bbox(bbox):
        center_x = bbox[0] + bbox[2] / 2
        center_y = bbox[1] + bbox[3] / 2
        size_x = bbox[2] - bbox[0]
        size_y = bbox[3] - bbox[1]
        return center_x, center_y, size_x, size_y
    """

    def run(self):
        # メインループ
        while not rospy.is_shutdown():
            # ループのスリープ
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = FaceDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
