#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

# グローバル変数としてボールの色、スタートフラグ、およびパブリッシャーを定義
ball_color = ""
start_flag = 0
color_publisher = None  # パブリッシャーのグローバル変数

def start_callback(data):
    global start_flag
    if data.data == "1":
        start_flag = 1

def color_callback(data):
    global ball_color, start_flag
    if start_flag == 0:
        rospy.loginfo("スタートしていません。")
        return
    
    if "ボール" in data.data or "的" in data.data:
        rospy.loginfo("ボールの色を検知しました。")
        update_colors(data.data)

def update_colors(text):
    global ball_color
    # 既に色が設定されている場合は更新しない
    if ball_color:
        return

    characters = [char for char in text]
    i = 0
    while characters[i:i+3] != ["ボ", "ー", "ル"]:
        if characters[i] in ["赤", "青", "黄"]:
            if characters[i] == "赤":
                ball_color = "red"
            elif characters[i] == "青":
                ball_color = "blue"
            elif characters[i] == "黄":
                ball_color = "yellow"
            rospy.loginfo("ボールの色が決定されました： %s", ball_color)
            break
        i += 1

def publish_color_event(event):
    global ball_color
    if ball_color:  # ball_colorが設定されていればパブリッシュ
        color_publisher.publish(ball_color)

def color_detector():
    global color_publisher
    rospy.init_node('color_detector', anonymous=True)
    
    # パブリッシャーを初期化
    color_publisher = rospy.Publisher('/ball_color', String, queue_size=10)

    rospy.Subscriber("/start_flag", String, start_callback)
    rospy.Subscriber("/speech_recog_result", String, color_callback)

    # 1秒ごとに現在のball_colorをパブリッシュする
    rospy.Timer(rospy.Duration(1), publish_color_event)
    
    rospy.spin()

if __name__ == '__main__':
    color_detector()
