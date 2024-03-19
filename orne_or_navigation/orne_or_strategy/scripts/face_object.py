#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import std_msgs.msg
from yolov5_pytorch_ros.msg import BoundingBoxes

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        rospy.loginfo("start")
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.camera_image_width = 640

        self.selected_color = None  # 初期状態で色が選択されていないことを示す

        # 検出結果を受け取るサブスクライバー
        self.sub_detection = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.detection_callback)

        # 色に関する指示を受け取るサブスクライバーを/ball_colorに変更
        self.sub_color = rospy.Subscriber('/ball_color', std_msgs.msg.String, self.color_callback)

    # 新しい色指示用のコールバック
        # 新しい色指示用のコールバック
    def color_callback(self, msg):
        color = msg.data  # 受け取ったメッセージから色を取得
        rospy.loginfo(f"Received ball color selection: {color}")  # 選択された色を表示
        self.selected_color = color

    def detection_callback(self, detection_msg):
        rospy.loginfo(f"Detection callback invoked with selected color: {self.selected_color}")
        if self.selected_color:
            for bbox in detection_msg.bounding_boxes:
                rospy.loginfo(f"Detected object class: {bbox.Class}")  # バウンディングボックスのクラス名を表示
                if f"ball_{self.selected_color}" == bbox.Class:
                    self.process_detected_object(bbox, self.selected_color)
                else:
                    rospy.loginfo(f"Detected object class does not match selected color: {bbox.Class}")
        else:
            rospy.loginfo("No ball color selected.")

    def process_detected_object(self, bbox, color):
        
        center_x = (bbox.xmin + bbox.xmax) / 2
        error_x = center_x - (self.camera_image_width / 2)
        rospy.loginfo(f"{color.capitalize()} object center X: {center_x}, Error X: {error_x}")
        twist_msg = Twist()
        twist_msg.angular.z = -error_x * 0.005
        self.cmd_vel_pub.publish(twist_msg)
        rospy.loginfo(f"Aligning robot with {color} object: angular.z = {twist_msg.angular.z}")

if __name__ == '__main__':
    try:
        controller = RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
