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

import tf
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
import numpy as np

# color definition
RED   = 1
GREEN = 2
BLUE  = 3

# PI
PI = 3.1415

# robot running coordinate in SEARCH MODE
basic_coordinate = np.array([
    # x, y, th
    [-0.4, 0.0, 0],  # 1
    [-0.9, 0.0, 0],  # 2
    [-0.9, 0.5, 0],  # 3
    [-0.9, -0.5, 0], # 4
    [-0.9, 0.0, 0],  # 5
    [0, -0.5, 0],    # 6
    [0, -0.5, PI],   # 7
    [0, -0.5, PI/2], # 8
    [0, -1.2, PI/2]] # 17
)


class AllSensorBot(object):
    basic_mode_process_step_idx = 0
    def __init__(self, 
                 use_lidar=False, use_camera=False, use_imu=False,
                 use_odom=False, use_joint_states=False):

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # lidar scan subscriber
        if use_lidar:
            self.scan = LaserScan()
            self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # camera subscribver
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
            self.img = None
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)

        # imu subscriber
        if use_imu:
            self.imu_sub = rospy.Subscriber('imu', Imu, self.imuCallback)

        # odom subscriber
        if use_odom:
            self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)

        # joint_states subscriber
        if use_joint_states:
            self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)

        # navigation publisher
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)


    def strategy(self):
        '''
        calc Twist and publish cmd_vel topic
        '''
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.func_basic()
            # update twist
            #twist = Twist()
            #twist.linear.x = 1; twist.linear.y = 0; twist.linear.z = 0
            #twist.angular.x = 1; twist.angular.y = 0; twist.angular.z = 0
            # publish twist topic
            #self.vel_pub.publish(twist)
            r.sleep()


    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        #rospy.loginfo(self.scan)

#image process
    def find_rect_of_target_color(self, image, color_type): # r:0, g:1, b:2
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]

        # red detection
        if color_type == RED:
            mask = np.zeros(h.shape, dtype=np.uint8)
            mask[((h < 20) | (h > 200)) & (s > 128)] = 255

        # blue detection
        if color_type == BLUE:
            lower_blue = np.array([130, 50, 50])
            upper_blue = np.array([200, 255, 255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # green detection
        if color_type == GREEN:
            lower_green = np.array([75, 50, 50])
            upper_green = np.array([110, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)

        # get contours
        img, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(np.array(rect))
        return rects

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        #bule detect
        #green detect
        #red detect
        img = self.img
        brects = self.find_rect_of_target_color(img,BLUE)
        grects = self.find_rect_of_target_color(img,GREEN)
        rrects = self.find_rect_of_target_color(img,RED)
        if len(brects) > 0:
            for rect in brects:
                cv2.rectangle(img, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
        if len(grects) > 0:
            for rect in grects:
                cv2.rectangle(img, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
        if len(rrects) > 0:
            for rect in rrects:
                cv2.rectangle(img, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)

        cv2.imshow("Image window", img)
        cv2.waitKey(1)

#navigation
    def func_basic(self):
        print("func_basic")
        twist = Twist()        
        # basic

        NextGoal_coor = basic_coordinate[ self.basic_mode_process_step_idx ]
        _x = NextGoal_coor[0]
        _y = NextGoal_coor[1]
        _th = NextGoal_coor[2]
        ret = self.setGoal(_x, _y, _th)

        if ret == 0:
            self.basic_mode_process_step_idx += 1
        else:
            # setGoal canceled
            print("setGoal ret:", ret)

        return 0

    # Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
    # Ref: https://github.com/hotic06/burger_war/blob/master/burger_war/scripts/navirun.py
    # RESPECT @hotic06
    # do following command first.
    #   $ roslaunch burger_navigation multi_robot_navigation_run.launch
    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        #TODO
        #goal.target_pose.header.frame_id = self.name + "/map"
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return -1

        get_state = self.client.get_state()
        print("wait", wait, "get_state", get_state)
        if get_state == 2:  # if send_goal is canceled
            return -1

        return 0

    def cancelGoal(self):
        self.client.cancel_goal()


    # imu call back sample
    # update imu state
    def imuCallback(self, data):
        self.imu = data
        #rospy.loginfo(self.imu)

    # odom call back sample
    # update odometry state
    def odomCallback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        #rospy.loginfo("odom pose_x: {}".format(self.pose_x))
        #rospy.loginfo("odom pose_y: {}".format(self.pose_y))

    # jointstate call back sample
    # update joint state
    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]
        #rospy.loginfo("joint_state R: {}".format(self.wheel_rot_r))
        #rospy.loginfo("joint_state L: {}".format(self.wheel_rot_l))

if __name__ == '__main__':
    rospy.init_node('all_sensor_sample')
    bot = AllSensorBot(use_lidar=True, use_camera=True, use_imu=True,
                       use_odom=True, use_joint_states=True)
    time.sleep(1.000) # wait for init complete
    bot.strategy()


