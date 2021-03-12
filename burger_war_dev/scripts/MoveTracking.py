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
import numpy as np

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

        # camera subscribver
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
            self.img1 = None
            self.img2 = None
            self.img3 = None
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

    def strategy(self):
        '''
        calc Twist and publish cmd_vel topic
        '''
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            # update twist
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0

            # publish twist topic
            self.vel_pub.publish(twist)
            if self.img3 is not None:
                bot.Move_Track()
            r.sleep()


    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        rospy.loginfo(self.scan)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
    # Temp. to get 2 images.
        try:
            self.img3 = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
        except CvBridgeError as e:
            rospy.logerr(e)

    def Move_Track(self):
            if self.img2 is None:
                self.img2 = self.img3
            if self.img1 is None:
                self.img1 = self.img2


            # make diff image
            gray1 = cv2.cvtColor(self.img1, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(self.img2, cv2.COLOR_BGR2GRAY)
            gray3 = cv2.cvtColor(self.img3, cv2.COLOR_BGR2GRAY)
            diff1 = cv2.absdiff(gray2, gray1)
            diff2 = cv2.absdiff(gray3, gray2)
            # get and image
            #im = cv2.bitwise_and(diff1,diff2)
            # apply thresh
            diff1_th = cv2.threshold(diff1, 60, 255, cv2.THRESH_BINARY)[1]
            diff2_th = cv2.threshold(diff2, 60, 255, cv2.THRESH_BINARY)[1]
            diff_and = cv2.bitwise_and(diff1_th,diff2_th)

            contours1, hierarchy1 = cv2.findContours(diff1_th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1:]
            contours2, hierarchy2 = cv2.findContours(diff2_th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1:]
            contours_and, hierarchy_and = cv2.findContours(diff_and, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1:]

            contours1 = list(filter(lambda x: cv2.contourArea(x) > 200, contours1))
            contours2 = list(filter(lambda x: cv2.contourArea(x) > 200, contours2))
            contours_and = list(filter(lambda x: cv2.contourArea(x) > 200, contours_and))
            #print("Contorus Num: " + str(len(contours1)) + str(len(contours2)))
            #print("Contorus Num: " + str(len(contours_and)))
            big_inx = 0 
            big_area = 0
            big_area_found = False
            for x in range(0, len(contours_and)):
                if len(contours_and[x]>0):
                    cont_area = cv2.contourArea(contours_and[x])
                    #print('x=%d area=%d' %(x,cont_area))
                    if cont_area > big_area:
                        big_area = cont_area
                        big_inx = x
                        big_area_found = True
                        #print('big_inx=%d big_area=%d' %(big_inx,big_area))
            if big_area_found == True:   
                rect_and = contours_and[big_inx]
                x,y,w,h = cv2.boundingRect(rect_and)
                #print("x=%x,y=%d,w=%d,h=%d" %(x,y,w,h))
                cv2.rectangle(self.img2,(x,y),(x+w,y+h),(0,255,0),5)

            cv2.imshow("Image window1", self.img2)
            #cv2.imshow("Image window2", self.img3)

            #cv2.imshow("Diff Image window1", diff1_th)
            #cv2.imshow("Diff Image window2", diff2_th)
            #cv2.imshow("Diff Image window1", diff_and)
            cv2.waitKey(1)
            self.img1 = self.img2
            self.img2 = self.img3

    # imu call back sample
    # update imu state
    def imuCallback(self, data):
        self.imu = data
        rospy.loginfo(self.imu)

    # odom call back sample
    # update odometry state
    def odomCallback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        rospy.loginfo("odom pose_x: {}".format(self.pose_x))
        rospy.loginfo("odom pose_y: {}".format(self.pose_y))

    # jointstate call back sample
    # update joint state
    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]
        rospy.loginfo("joint_state R: {}".format(self.wheel_rot_r))
        rospy.loginfo("joint_state L: {}".format(self.wheel_rot_l))

if __name__ == '__main__':
    rospy.init_node('all_sensor_sample')
    bot = AllSensorBot(use_lidar=False, use_camera=True, use_imu=False,
                       use_odom=False, use_joint_states=False)
    bot.strategy()


