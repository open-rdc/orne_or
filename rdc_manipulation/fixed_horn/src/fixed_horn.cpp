#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float64.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_msgs");
  ros::NodeHandle n;
  ros::Publisher pub_tilt = n.advertise<std_msgs::Float64>("/joint_tilt_controller/command", 10);
  ros::Publisher pub_pan = n.advertise<std_msgs::Float64>("/joint_pan_controller/command", 10);
  ros::Rate loop_rate(5);
  while(ros::ok())
  {
    std_msgs::Float64 msg_tilt;
    std_msgs::Float64 msg_pan;
    msg_tilt.data = 0.0;
    msg_pan.data = 0.0;
    float a = msg_tilt.data;
    float b = msg_pan.data;
    ROS_INFO("joint_tilt = %f, joint_pan = %f", a, b);
    pub_tilt.publish(msg_tilt);
    pub_pan.publish(msg_pan);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
