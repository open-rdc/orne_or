#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from yolov5_pytorch_ros.msg import BoundingBoxes

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        # cmd_velパブリッシャーの設定
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # カメラ画像の幅を仮定する（実際にはカメラから取得する）
        self.camera_image_width = 640  # この値は使用しているカメラによって変わります
        # それぞれの色のトピックに対するサブスクライバーを設定
        self.sub_red = rospy.Subscriber('/detected_objects/red', BoundingBoxes, self.callback, callback_args='red')
        self.sub_blue = rospy.Subscriber('/detected_objects/blue', BoundingBoxes, self.callback, callback_args='blue')
        self.sub_yellow = rospy.Subscriber('/detected_objects/yellow', BoundingBoxes, self.callback, callback_args='yellow')

    def callback(self, data, color):
        rospy.loginfo(f"{color.capitalize()} object detected with bounding boxes: {data.bounding_boxes}")
        self.align_robot_with_object(data, color)

    def align_robot_with_object(self, detection_msg, color):
        target_object = f"ball_{color}"
        for box in detection_msg.bounding_boxes:
            if box.Class == target_object:
                center_x = (box.xmin + box.xmax) / 2
                error_x = center_x - (self.camera_image_width / 2)
                # 追加: バウンディングボックスの中心座標とカメラ座標の中心との差を表示
                rospy.loginfo(f"{color.capitalize()} object center X: {center_x}, Error X: {error_x}")
                twist_msg = Twist()
                twist_msg.angular.z = -error_x * 0.005
                self.cmd_vel_pub.publish(twist_msg)
                rospy.loginfo(f"Aligning robot with {color} object: angular.z = {twist_msg.angular.z}")
                break


if __name__ == '__main__':
    try:
        controller = RobotController()
        rospy.spin()  # Keep the program running until a shutdown signal is received
    except rospy.ROSInterruptException:
        pass
