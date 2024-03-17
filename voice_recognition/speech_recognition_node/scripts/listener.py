#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

ball_color = None  # ボールの色
target_color = None  # 的の色
x = 0

def update_colors(text):
    global ball_color, target_color, x
    
    characters = [char for char in text]
    
    if "スタート" in text or "開始" in text:
        print("開始します。")
        x = 1  # スタートが検出された場合、xを1に設定
    print(x)
    
    # 3文字ずつのスライディングウィンドウで "ボール" を探す
    if ("ボール" in text or "的" in text) and x == 1:
        i = 0

        while characters[i:i+3] != ["ボ", "ー", "ル"]:
            if characters[i] in ["赤", "青", "黄"]:
                ball_color = characters[i]
            i += 1
        print("ボールの色：", ball_color)
        
        i += 3  

        while characters[i] != "的":
            if characters[i] in ["赤", "青", "黄"]:
                target_color = characters[i]
            i += 1
        print("的の色:", target_color)
    elif x != 1:
        print("スタートしていません。")
    else:
        print("ボールまたは的が見つかりませんでした。次の出力を処理します。")

def callback(data):
    rospy.loginfo("Received speech recognition result: %s", data.data)
    update_colors(data.data)  # テキストから色を更新

if __name__ == '__main__':
    rospy.init_node('speech_recognition_listener', anonymous=True)
    rospy.Subscriber("/speech_recog_result", String, callback)
    rospy.spin()

