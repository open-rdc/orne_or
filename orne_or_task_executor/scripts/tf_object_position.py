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
            if not self.listener.canTransform('/base_link', point_msg.header.frame_id, rospy.Time(0)):
                rospy.logwarn("Transformation from %s to /base_link not ready.", point_msg.header.frame_id)
                return
            transformed_point = self.listener.transformPoint('/base_link', point_msg)
            transformed_point.point.x /= 1000
            transformed_point.point.y /= 1000
            transformed_point.point.z += 1100
            transformed_point.point.z /= 1000
            rospy.loginfo("Transformed Point: %s", transformed_point.point)
            self.publisher.publish(transformed_point)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF error: %s", e)

if __name__ == '__main__':
    rospy.init_node('point_transformer')
    transformer = PointTransformer()
    rospy.spin()

