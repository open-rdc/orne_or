#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from yolov5_pytorch_ros.msg import BoundingBoxes

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        
        # cmd_velパブリッシャーの設定
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # それぞれの色のトピックに対するサブスクライバーを設定
        self.sub_red = rospy.Subscriber('/speaker/red', BoundingBoxes, self.callback, callback_args='red')
        self.sub_blue = rospy.Subscriber('/speaker/blue', BoundingBoxes, self.callback, callback_args='blue')
        self.sub_yellow = rospy.Subscriber('/speaker/yellow', BoundingBoxes, self.callback, callback_args='yellow')
        
        rospy.spin()

    def callback(self, data, color):
        # 色に基づいて処理を分岐
        if color == 'red':
            rospy.loginfo("Red object detected")
        elif color == 'blue':
            rospy.loginfo("Blue object detected")
        elif color == 'yellow':
            rospy.loginfo("Yellow object detected")
        
        # バウンディングボックスの中心座標とカメラ画像のx軸の中心座標を合わせるロジックを実装
        self.align_robot_with_object(data, color)
        
    def align_robot_with_object(self, detection_msg, color):
        # ここにバウンディングボックスの中心とカメラ画像の中心を合わせるロジックを実装します。
        # このサンプルでは、具体的な動きの実装は省略しています。
        pass

if __name__ == '__main__':
    try:
        RobotController()
    except rospy.ROSInterruptException:
        pass
