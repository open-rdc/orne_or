#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from yolov5_pytorch_ros.msg import BoundingBoxes

class Object3DPositionPublisher:
    def __init__(self):
        rospy.init_node('object_3d_position_publisher', anonymous=True)
        
        # カメラモデル
        self.camera_model = PinholeCameraModel()
        
        # サブスクライバーとパブリッシャーの設定
        self.depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.info_sub = message_filters.Subscriber('/camera/depth/camera_info', CameraInfo)
        self.bbox_sub = message_filters.Subscriber('/bounding_boxes', BoundingBoxes)
        self.position_pub = rospy.Publisher('/object_position', PointStamped, queue_size=10)
        
        # 同期処理
        ts = message_filters.ApproximateTimeSynchronizer([self.bbox_sub, self.depth_sub, self.info_sub], 10, 0.5)
        ts.registerCallback(self.callback)

        # CvBridge
        self.bridge = CvBridge()

    def callback(self, bbox_msg, depth_msg, info_msg):
        # カメラモデルの更新
        self.camera_model.fromCameraInfo(info_msg)
        
        # 深度画像を取得
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg)
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 最初に検出された物体の中心座標を使用
        if bbox_msg.bounding_boxes:
            bbox = bbox_msg.bounding_boxes[0]
            x = int((bbox.xmin + bbox.xmax) / 2.0)
            y = int((bbox.ymin + bbox.ymax) / 2.0)

            # 深度値を取得
            depth = depth_image[y, x]

            # 3D点を計算
            ray = self.camera_model.projectPixelTo3dRay((x, y))
            normalized_ray = np.array(ray) / np.linalg.norm(ray)  # 単位ベクトルに正規化
            point = depth * normalized_ray  # 実際の座標を計算

            # 座標をパブリッシュ
            point_stamped = PointStamped()
            point_stamped.header.stamp = rospy.Time.now()
            point_stamped.header.frame_id = depth_msg.header.frame_id
            point_stamped.point.x = point[0]
            point_stamped.point.y = point[1]
            point_stamped.point.z = point[2]
            self.position_pub.publish(point_stamped)
            rospy.loginfo(f"Published 3D object position: {point_stamped}")

if __name__ == '__main__':
    try:
        node = Object3DPositionPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
