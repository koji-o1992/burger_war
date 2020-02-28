#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This is ALL SENSOR use node.
Mainly echo sensor value in tarminal.
Please Use for your script base.

by Takuya Yamaguchi @dashimaki360
'''

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy
import inspect

class AllSensorBot(object):
    def __init__(self, 
                 use_lidar=False, use_camera=False, use_imu=False,
                 use_odom=False, use_joint_states=False):

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # lidar scan subscriber
        if use_lidar:
            self.scan = LaserScan()
            self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)
            # for key in self.__dict__.keys():
            #     print(key)
            # for key, value in self.__dict__.items():
            #     print(key, ':', value)

            
        # camera subscribver
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
            self.img = None
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)

            # for key, value in self.__dict__.items():
            #     print(key, ':', value)


        # imu subscriber
        if use_imu:
            self.imu_sub = rospy.Subscriber('imu', Imu, self.imuCallback)

            # for key, value in self.__dict__.items():
            #     print(key, ':', value)

        # odom subscriber
        if use_odom:
            self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)

            # for key, value in self.__dict__.items():
            #     print(key, ':', value)

        # joint_states subscriber
        if use_joint_states:
            self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)

            # for key, value in self.__dict__.items():
            #     print(key, ':', value)

                
    def strategy(self):
        '''
        calc Twist and publish cmd_vel topic
        '''
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            # update twist
            twist = Twist()
            twist.linear.x = 0; # 0.1;
            twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

            # for key, value in self.__dict__.items():
            #     print(key, ':', value)

            # print(inspect.getmembers(self.scan))
            
            # if use_lidar:
            # 最大値のインデックス取得
            max_idx = 0
            min_idx = 0
            if(len(self.scan.intensities) != 0) : 
                max_idx = numpy.argmax(self.scan.intensities)
                min_idx = numpy.argmin(self.scan.intensities)
                print('max_idx')
                print(max_idx)
                # print(self.scan.ranges(max_idx))
                print(max(self.scan.intensities))
                # print('min_idx')
                # print(min_idx)
                # # print(self.scan.ranges(min_idx))
                # print(min(self.scan.intensities))
            # print(self.scan.intensities)
            else :
                max_idx = 0
                min_idx = 0

            # print("max_idx:"  max_idx)
            if(max_idx > 90 and max_idx < 270) :
                twist.linear.x = -1
                print('back')
            else : 
                twist.linear.x = 1
                print('forward')

            if(max_idx < 180) :
                twist.angular.z = -1
                print('right')
            else :
                twist.angular.z = 1                
                print('left')
                
                
                
                

            # print("max_idx:"  max_idx)
            # if(max_idx == 0) :
            #     twist.linear.x = 0.1
            #     twist.angular.z = 0
            #     print('c')
            # elif(max_idx < 180) :
            #     twist.linear.x = 0
            #     twist.angular.z = -1
            #     print('r')
            # else :
            #     twist.linear.x = 0
            #     twist.angular.z = 1
            #     print('l')                

            
            # publish twist topic
            self.vel_pub.publish(twist)
            r.sleep()

            

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        # rospy.loginfo(self.scan)

        # print(self.scan.angle_min)
        # print(self.scan.angle_max)
        # print(len(self.scan.ranges))
        # print(len(self.scan.intensities))
        # print(max(self.scan.intensities))
        # for key in self.__dict__.keys():
        #     print(key)
        # for key, value in self.__dict__.items():
        #     print(key, ':', value)

            
        
        

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # cv2.imshow("Image window", self.img)
        # cv2.waitKey(1)

    # imu call back sample
    # update imu state
    def imuCallback(self, data):
        self.imu = data
        # rospy.loginfo(self.imu)

    # odom call back sample
    # update odometry state
    def odomCallback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        # rospy.loginfo("odom pose_x: {}".format(self.pose_x))
        # rospy.loginfo("odom pose_y: {}".format(self.pose_y))

    # jointstate call back sample
    # update joint state
    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]
        # rospy.loginfo("joint_state R: {}".format(self.wheel_rot_r))
        # rospy.loginfo("joint_state L: {}".format(self.wheel_rot_l))

if __name__ == '__main__':
    rospy.init_node('all_sensor_sample')
    bot = AllSensorBot(use_lidar=True, use_camera=True, use_imu=True,
                       use_odom=True, use_joint_states=True)
    bot.strategy()


