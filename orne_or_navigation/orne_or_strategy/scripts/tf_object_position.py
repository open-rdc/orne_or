#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

def callback(point_stamped):
    try:
        # カメラ座標系からロボット座標系への変換を試みます
        transform = tf_buffer.lookup_transform('base_link',  # 目標フレーム
                                               point_stamped.header.frame_id,  # ソースフレーム
                                               rospy.Time(0),  # 最新のトランスフォームを取得
                                               rospy.Duration(1.0))  # タイムアウト時間
        transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        
        # 変換後の座標をパブリッシュ
        pub.publish(transformed_point)

        # 変換後の座標を表示
        rospy.loginfo("Transformed Point: x=%.2f y=%.2f z=%.2f" %
                      (transformed_point.point.x,
                       transformed_point.point.y,
                       transformed_point.point.z))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr('TF2 transform error: %s' % str(e))

def main():
    rospy.init_node('object_position_transformer')
    
    global tf_buffer, pub
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # 変換後の座標をパブリッシュするためのパブリッシャーを定義
    pub = rospy.Publisher('/tf_object_position', PointStamped, queue_size=10)
    
    rospy.Subscriber('/object_position', PointStamped, callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()
