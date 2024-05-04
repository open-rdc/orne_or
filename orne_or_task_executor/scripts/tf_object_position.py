#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PointStamped

class PointTransformer:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.publisher = rospy.Publisher('/tf_object_position', PointStamped, queue_size=10)
        rospy.Subscriber('/object_position', PointStamped, self.callback)

    def callback(self, point_msg):
        try:
            # カメラ座標系からベース座標系への変換を待つ
            self.listener.waitForTransform('/base_link', point_msg.header.frame_id, rospy.Time(0), rospy.Duration(4.0))
            # 座標変換を実行
            transformed_point = self.listener.transformPoint('/base_link', point_msg)
            transformed_point.point.x /= 1000
            transformed_point.point.y /= 1000
            transformed_point.point.z += 1100
            transformed_point.point.z /= 1000
            rospy.loginfo("Transformed Point: %s", transformed_point.point)
            # 変換されたポイントをパブリッシュ
            self.publisher.publish(transformed_point)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF error: %s", e)

if __name__ == '__main__':
    rospy.init_node('point_transformer')
    transformer = PointTransformer()
    rospy.spin()

