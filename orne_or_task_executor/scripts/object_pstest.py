#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point

def publish_point():
    rospy.init_node('point_publisher', anonymous=True)  # ノードを初期化
    pub = rospy.Publisher('point_topic', Point, queue_size=10)  # パブリッシャーを設定
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        point_msg = Point()  # Point メッセージオブジェクトを作成
        point_msg.x = 0.12
        point_msg.y = -0.26
        point_msg.z = 0.54
        
        # rospy.loginfo("Publishing: {}".format(point_msg))  # ログ情報を出力
        pub.publish(point_msg)  # トピックにメッセージをパブリッシュ
        rate.sleep()  # 次のループまで待機

if __name__ == '__main__':
    try:
        publish_point()
    except rospy.ROSInterruptException:
        pass

