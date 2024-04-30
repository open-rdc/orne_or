#!/usr/bin/env python3
# coding:UTF-8

import rospy
from geometry_msgs.msg import PointStamped, Point

def main():
    rospy.init_node("point_publisher")
    pub = rospy.Publisher('/target_point', PointStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # 座標を作成
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = "world"  # フレームIDを指定
        point_msg.point = Point(0.4517, 0.0318, 0.568653)  # 座標を指定

        # メッセージをPublish
        pub.publish(point_msg)
        rate.sleep()

if __name__ == "__main__":
    main()
