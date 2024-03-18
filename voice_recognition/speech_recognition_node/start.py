#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

x = 0  # スタートを検知するフラグ

def start_callback(data):
    global x
    if "スタート" in data.data or "開始" in data.data:
        print("スタートを検知しました。")
        x = 1

def start_detector():
    rospy.init_node('start_detector', anonymous=True)
    rospy.Subscriber("/speech_recog_result", String, start_callback)
    pub = rospy.Publisher("/start_flag", String, queue_size=10)  # トピックの定義
    #rospy.Subscriber("/speech_recog_result", String, result_callback)  # 出力を受け取るサブスクライバーの追加
    rate = rospy.Rate(5)  # ループの頻度を定義
    
    start_pub = rospy.Publisher("/start_flag", String, queue_size=1)
    
    while not rospy.is_shutdown():
        start_pub.publish(str(x))  # xの値をトピックにパブリッシュ
        rate.sleep()

if __name__ == '__main__':
    start_detector()
