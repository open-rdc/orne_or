#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

# グローバル変数としてボールの色とスタートフラグを定義
ball_color = ""
start_flag = 0

def start_callback(data):
    global start_flag
    # start.pyからのメッセージに応じてスタートフラグを更新
    if data.data == "1":
        start_flag = 1

def color_callback(data):
    global ball_color, start_flag
    # スタートが検知されていない場合は処理をスキップ
    if start_flag == 0:
        rospy.loginfo("スタートしていません。")
        return
    
    # ボールか的の色を検知したときに呼ばれる関数
    if "ボール" in data.data or "的" in data.data:
        rospy.loginfo("ボールの色を検知しました。")
        update_colors(data.data)

def update_colors(text):
    global ball_color
    characters = [char for char in text]
    i = 0
    while characters[i:i+3] != ["ボ", "ー", "ル"]:
        if characters[i] == "赤":
            ball_color = "red"
        elif characters[i] == "青":
            ball_color = "blue"
        elif characters[i] == "黄":
            ball_color = "yellow"
        i += 1
    
    rospy.loginfo("ボールの色： %s", ball_color)
    # 色の値をパブリッシュする
    publish_color_value(ball_color)

def publish_color_value(color):
    # ボールの色を/ball_colorトピックにパブリッシュする
    pub = rospy.Publisher('/ball_color', String, queue_size=10)
    rospy.sleep(1)  # パブリッシャがセットアップされるのを待つ
    pub.publish(color)

def color_detector():
    rospy.init_node('color_detector', anonymous=True)
    
    # start.pyからのトピックを受信するコールバック関数を登録
    rospy.Subscriber("/start_flag", String, start_callback)
    
    # 色を受信するトピックのコールバック関数を登録
    rospy.Subscriber("/speech_recog_result", String, color_callback)
    
    rospy.spin()

if __name__ == '__main__':
    color_detector()
