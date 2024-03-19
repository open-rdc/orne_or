#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import std_msgs.msg
from yolov5_pytorch_ros.msg import BoundingBoxes

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        rospy.loginfo("Robot Controller Node started")
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.camera_image_width = 640

        self.selected_color = None

        self.sub_detection = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.detection_callback)
        self.sub_color = rospy.Subscriber('/ball_color', std_msgs.msg.String, self.color_callback)

    def color_callback(self, msg):
        self.selected_color = msg.data
        rospy.loginfo(f"Received ball color selection: {self.selected_color}")

    def detection_callback(self, detection_msg):
        rospy.loginfo(f"Detection callback invoked with selected color: {self.selected_color}")
        if self.selected_color:
            for bbox in detection_msg.bounding_boxes:
                rospy.loginfo(f"Detected object class: {bbox.Class}")
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
        
        # 速度の設定を確認
        twist_msg = Twist()
        twist_msg.angular.z = -error_x * 0.005
        rospy.loginfo(f"Publishing to /cmd_vel with angular.z = {twist_msg.angular.z}")
        self.cmd_vel_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
