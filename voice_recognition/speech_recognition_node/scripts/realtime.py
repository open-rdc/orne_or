#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import speech_recognition as sr
import os

def speech_recognition_node():
    # ファイルが存在する場合は削除する
    if os.path.exists("recognized_text.txt"):
        os.remove("recognized_text.txt")
        rospy.loginfo("Existing file 'recognized_text.txt' removed.")

    # ROSノードの初期化
    rospy.init_node("speech_recognition", anonymous=True)

    # パブリッシャーの作成
    result_pub = rospy.Publisher("/speech_recog_result", String, queue_size=1)

    # 音声認識器を初期化
    recognizer = sr.Recognizer()

    # マイクからの音声をリアルタイムで認識
    with sr.Microphone() as source:
        # マイクから音声をリアルタイムで取得
        while not rospy.is_shutdown():
            audio = recognizer.listen(source, timeout=None)

            try:
                # Googleの音声認識APIを使用して音声をテキストに変換
                text = recognizer.recognize_google(audio, language='ja-JP')
                rospy.loginfo("You said: %s", text)
                
                # ROSトピックに音声認識結果をパブリッシュ
                result_pub.publish(text)

                # テキストをファイルに書き込む
                with open("recognized_text.txt", "a") as file:
                    file.write(text)

            except sr.UnknownValueError:
                rospy.logwarn("Google Speech Recognition could not understand audio")
            except sr.RequestError as e:
                rospy.logerr("Could not request results from Google Speech Recognition service; {0}".format(e))

if __name__ == '__main__':
    try:
        speech_recognition_node()
    except rospy.ROSInterruptException:
        pass

