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
        if characters[i] in ["赤", "青", "黄"]:
            ball_color = characters[i]
            rospy.loginfo("ボールの色： %s", ball_color)
        i += 1
    
    if ball_color == "赤":
        pub.publish("/speaker/red")
        publish_color_value("赤")  # メッセージをパブリッシュする
    elif ball_color == "青":
        pub.publish("/speaker/blue")
        publish_color_value("青")  # メッセージをパブリッシュする
    elif ball_color == "黄":
        pub.publish("/speaker/yellow")
        publish_color_value("黄")  # メッセージをパブリッシュする

def publish_color_value(color):
    pub = rospy.Publisher('/color_values', String, queue_size=10)
    rospy.loginfo("Publishing color value: %s", color)
    pub.publish(color)

def color_detector():
    rospy.init_node('color_detector', anonymous=True)
    
    # パブリッシャーをグローバル変数として定義
    global pub
    pub = rospy.Publisher('/speaker', String, queue_size=10)
    
    # start.pyからのトピックを受信するコールバック関数を登録
    rospy.Subscriber("/start_flag", String, start_callback)
    
    # 色を受信するトピックのコールバック関数を登録
    rospy.Subscriber("/speech_recog_result", String, color_callback)
    
    # ループの周期を設定
    rate = rospy.Rate(10)  # 10Hz
    
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    color_detector()

