#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
import tf
from geometry_msgs.msg import Quaternion
import math
from std_srvs.srv import Trigger
from fulanghua_srvs.srv import Pose
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import yaml
import roslib.packages



class TsukubaChallengeStrategy:
    def __init__(self):
        rospy.init_node('tsukuba_challenge_strategy')
        self.start_nav_server = rospy.Service('start_nav', Trigger, self.start_nav_callback)
        self.resume_nav_server = rospy.Service('resume_nav', Trigger, self.resume_nav_callback)
        self.scan_sub = rospy.Subscriber('/scan',LaserScan ,self.scan_callback)
        self.vel_scan_sub = rospy.Subscriber('/vel_scan',LaserScan ,self.vel_scan_callback)
        self.current_sp = 0
        self.strategy_state= rospy.get_param('state')
        self.sensor_data = 0
        self.vel_sensor_data = 0


        self.world_frame = rospy.get_param('~world_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')

        file_name = rospy.get_param('~filename',roslib.packages.get_pkg_dir('orne_box_strategy')+'/suspend_cfg/suspend.yaml')

        print("read suspend file: " + file_name)
        f = open(file_name,'r+')
        yaml_data = yaml.load(f)
        self.suspend_size = len(yaml_data['suspend_pose'])

        self.suspend_pose = []
        self.resume_pose = []
        self.line_tracking = []


        for i in range(self.suspend_size):

            self.suspend_pose.append(geometry_msgs.msg.Pose())
            self.resume_pose.append(geometry_msgs.msg.Pose())
            self.line_tracking.append(0)


            self.line_tracking[i] = yaml_data['suspend_pose'][i]['pose']['line_tracking']
            self.suspend_pose[i].position.x = yaml_data['suspend_pose'][i]['pose']['position']['x']
            self.suspend_pose[i].position.y = yaml_data['suspend_pose'][i]['pose']['position']['y']
            self.suspend_pose[i].position.z = yaml_data['suspend_pose'][i]['pose']['position']['z']
            self.suspend_pose[i].orientation.x = yaml_data['suspend_pose'][i]['pose']['orientation']['x']
            self.suspend_pose[i].orientation.y = yaml_data['suspend_pose'][i]['pose']['orientation']['y']
            self.suspend_pose[i].orientation.z = yaml_data['suspend_pose'][i]['pose']['orientation']['z']
            self.suspend_pose[i].orientation.w = yaml_data['suspend_pose'][i]['pose']['orientation']['w']

            self.resume_pose[i].position.x = yaml_data['resume_pose'][i]['pose']['position']['x']
            self.resume_pose[i].position.y = yaml_data['resume_pose'][i]['pose']['position']['y']
            self.resume_pose[i].position.z = yaml_data['resume_pose'][i]['pose']['position']['z']
            self.resume_pose[i].orientation.x = yaml_data['resume_pose'][i]['pose']['orientation']['x']
            self.resume_pose[i].orientation.y = yaml_data['resume_pose'][i]['pose']['orientation']['y']
            self.resume_pose[i].orientation.z = yaml_data['resume_pose'][i]['pose']['orientation']['z']
            self.resume_pose[i].orientation.w = yaml_data['resume_pose'][i]['pose']['orientation']['w']

            print "suspend_pose" + str(i+1) + " = " + str(self.suspend_pose[0])
            print "resume_pose" + str(i+1) + " = " + str(self.resume_pose[0])

        self.tf_listener = tf.TransformListener()

    def start_nav_callback(self, req):
        try:
            rospy.loginfo("strategy: start_nav_callback")
            rospy.wait_for_service('start_wp_nav')
            start_wp_nav = rospy.ServiceProxy('start_wp_nav', Trigger)
            return start_wp_nav()
            #rospy.loginfo(start_wp_nav())
            #return (True, "successful")
        except rospy.ServiceException, e:
            print "error: %s" % e

    def resume_nav_callback(self, req):
        try:
            rospy.loginfo('strategy: resume_nav_callback')
            rospy.wait_for_service('resume_wp_pose')
            resume_wp_nav = rospy.ServiceProxy('resume_wp_pose', Pose)
            return resume_wp_nav(self.resume_pose[self.current_sp-1])
        except rospy.ServiceException, e:
            print "error: %s" % e

    def get_yaw_from_quaternion(self, quaternion):
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        print "yaw = " + str(math.degrees(e[2]))
        return e[2]



    def pub_vel(self):
        #suspend_point:move_baseを停止する位置
        #rviz上のresume_wp_nav:はmove_baseが停止している時に有効,resume_pointに一番近いway_pointをmove_baseに渡しmove_baseを再開する
        #関数内で使っているラインはsuspend_pointとresume_pointを結んだ線
        rate = rospy.Rate(10)
        
        pub = rospy.Publisher('/icart_mini/cmd_vel', Twist, queue_size=10)
        vel = Twist()
        vel.linear.x = 0.5  #速度
        vel.angular.z = 0   #角速度
    
        alpha = -2.0    #角速度を決めるときにラインの角度とロボットの姿勢の差にかかる係数
        beta = -4.0     #ラインとロボットの距離にかかる係数

        angular_diff = 0    #ラインとロボットの姿勢の差
        vertical_diff = 0   #ラインとロボットの距離

        dist = 10   #resume_poseとロボットの距離
                
        #resume_poseとロボットのの距離が1mより大きい間ループ
        while dist > 1:
            rate.sleep()
            try:

                #ロボットの位置、姿勢を取得
                (trans, rot) = self.tf_listener.lookupTransform(self.world_frame, self.robot_frame, rospy.Time(0))
                x = trans[0]
                y = trans[1]

                ori_x = rot[0]
                ori_y = rot[1]
                ori_z = rot[2]
                ori_w = rot[3]

                #resume_poseとロボットの距離
                dist = math.sqrt(math.pow(x - self.resume_pose[self.current_sp].position.x, 2) + math.pow(y - self.resume_pose[self.current_sp].position.y, 2))

                #ロボットの姿勢(quaternionからeulerに変換)
                robot_angular = tf.transformations.euler_from_quaternion((ori_x, ori_y, ori_z, ori_w))
                #ラインの角度
                line_angular =  math.atan2(self.resume_pose[self.current_sp].position.y - self.suspend_pose[self.current_sp].position.y, self.resume_pose[self.current_sp].position.x - self.suspend_pose[self.current_sp].position.x)
                #ラインとロボットの角度の差
                angular_diff = robot_angular[2] - line_angular
                if angular_diff < -math.pi:
                    angular_diff = angular_diff + 2*math.pi
                #suspend_poseとロボットの位置を結んだ線の角度
                point_angular = math.atan2(y - self.suspend_pose[self.current_sp].position.y,x - self.suspend_pose[self.current_sp].position.x) - line_angular
                #ロボットとラインの距離
                vertical_diff = math.sin(point_angular) * math.sqrt(math.pow(x - self.suspend_pose[self.current_sp].position.x, 2) + math.pow(y - self.suspend_pose[self.current_sp].position.y, 2))



                #scanまたはvel_scanの値が0.01~1.5の時ループ
                while (self.sensor_data < 1.5 and self.sensor_data > 0.01) or (self.vel_sensor_data < 1.5 and self.vel_sensor_data > 0.01):
                    
                    vel.linear.x = 0
                    vel.angular.z = 0
                    pub.publish(vel)
                    print('scan',self.sensor_data)
                    print('vel_scan',self.vel_sensor_data)

                    #print("stop")
                    #print('robot_angular:',robot_angular,' line_angular:',line_angular,' angular_diff',angular_diff)


                #角速度を求める
                vel.angular.z = alpha * angular_diff + beta * vertical_diff
                if vel.angular.z > math.pi/2:
                    vel.angular.z = math.pi/2
                if vel.angular.z < -math.pi/2:
                    vel.angular.z = -math.pi/2
                   
                vel.linear.x = 0.5
                pub.publish(vel)
                #print('dist:',dist,' angular_diff', angular_diff, 'vertical_diff', vertical_diff)
 

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


    #laser_scan型から90-degree ~ 90+degreeの範囲で最小の値を返す
    def min_scan_val(self,msg,degree=5):

        one_data_rad = (msg.angle_max - msg.angle_min)/len(msg.ranges)
        n = math.ceil(math.pi * degree / 180 / one_data_rad)
        start = int(len(msg.ranges)/2-n)
        finish = int(len(msg.ranges)/2+n+1)

        number_list = range(start,finish)
        data_list = []
        for i in number_list:
            data_list.append(msg.ranges[i])

        return min(data_list)
        
        
    def scan_callback(self,msg):
        self.sensor_data = self.min_scan_val(msg)
        
    def vel_scan_callback(self,msg):
        self.vel_sensor_data = self.min_scan_val(msg)
 

    def spin(self):
        rate = rospy.Rate(10)

        while  not rospy.is_shutdown():
            rate.sleep()
            if self.strategy_state is True:
                try:
                    (trans, rot) = self.tf_listener.lookupTransform(self.world_frame, self.robot_frame, rospy.Time(0))
                    x = trans[0]
                    y = trans[1]

                    ori_x = rot[0]
                    ori_y = rot[1]
                    ori_z = rot[2]
                    ori_w = rot[3]

                    if self.current_sp < self.suspend_size:
                        dist = math.sqrt(math.pow(x - self.suspend_pose[self.current_sp].position.x, 2) + math.pow(y - self.suspend_pose[self.current_sp].position.y, 2))
                        print "robot_gl = ("  + str(x) + ", " + str(y) + ")"
                        print "dist = " + str(dist)
                        print "curennt_sp =  " + str(self.current_sp)
                    else :
                        print "all suspend pose finished"

                    if(dist < 1.5) and self.current_sp  <  self.suspend_size:
                        # self.get_yaw_from_quaternion(Quaternion(ori_x,ori_y,ori_z,ori_w))
                        # self.get_yaw_from_quaternion(Quaternion(self.suspend_pose[self.current_sp].orientation.x,self.suspend_pose[self.current_sp].orientation.y,self.suspend_pose[self.current_sp].orientation.y,self.suspend_pose[self.current_sp].orientation.w))

                        # input current robot angle
                        self.suspend_pose[self.current_sp].orientation.x = ori_x
                        self.suspend_pose[self.current_sp].orientation.y = ori_y
                        self.suspend_pose[self.current_sp].orientation.z = ori_z
                        self.suspend_pose[self.current_sp].orientation.w = ori_w

                        rospy.loginfo('strategy: send suspending request')
                        rospy.wait_for_service('suspend_wp_pose')
                        suspend_wp_nav = rospy.ServiceProxy('suspend_wp_pose', Pose)
                        print "suspend_wp_nav() = " + str(suspend_wp_nav(self.suspend_pose[self.current_sp]))


                        if self.line_tracking[self.current_sp] is 1: 
                            self.pub_vel()
                        self.current_sp += 1


                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
            else :
                #print " No suspend Pose "
                pass


if __name__ == '__main__':
    strategy = TsukubaChallengeStrategy()
    strategy.spin()
