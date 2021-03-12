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
import math


class AllSensorBot(object):
    def __init__(self, 
                 use_lidar=False, use_camera=False, use_imu=False,
                 use_odom=False, use_joint_states=False):

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # lidar scan subscriber
        if use_lidar:
            self.scan = LaserScan()
            self.scanned = LaserScan()
            self.RadarRatio = 50
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

    def strategy(self):
        '''
        calc Twist and publish cmd_vel topic
        '''
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            # update twist
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            if len(self.scan.ranges) != 0:
                bot.Radar()
            # publish twist topic
            self.vel_pub.publish(twist)
            r.sleep()

    def Radar(self):
        """
        Radar map from LIDAR
        """
        print "Radar func"
        if len(self.scanned.ranges) == 0:
            self.scanned.ranges = self.scan.ranges[:]
        npScanRanges = np.array(self.scan.ranges)
        npScannedRanges = np.array(self.scanned.ranges)
        npSubRanges = abs(npScanRanges - npScannedRanges)
        for i in range(len(npSubRanges)):
            if npSubRanges[i] < 0.15:
                npSubRanges[i] = 0
            else:
                npSubRanges[i] = 1
        npMaskedRanges = npScanRanges*npSubRanges
        """
            if npSubRanges[i] != 0:
                print "i=%d Range=%f" %(i,npSubRanges[i])
        print npSubRanges
        """
        """
        Create blank image with 701x701[pixel]
        """
        height = int(self.scan.range_max * self.RadarRatio * 2 + 1)
        width = int(self.scan.range_max * self.RadarRatio * 2 + 1)
        radar = np.ones((height,width,3),np.uint8)*40
        origin_x = int(self.scan.range_max * self.RadarRatio)
        origin_y = int(self.scan.range_max * self.RadarRatio)
        #radar.itemset((origin_x,origin_y,2),255)
        #radar[origin_x,origin_y] = [255,255,255]
        
        for n in range(0,width):
            radar.itemset((origin_y,n,2),255)
            radar.itemset((n,origin_x,2),255)
        
         
        for i in range(len(npMaskedRanges)):
            if npMaskedRanges[i] != 0:
                if i <= 90:
                    ang = np.deg2rad(90 - i)
                    x = origin_x - int(self.RadarRatio * npMaskedRanges[i] * math.cos(ang))
                    y = origin_y - int(self.RadarRatio * npMaskedRanges[i] * math.sin(ang))
                    print "i:%d ang:%f x:%d y:%d range:%f" %(i, np.rad2deg(ang),x,y,npMaskedRanges[i])
                elif i > 90 and i <= 180:
                    ang = np.deg2rad(i - 90)
                    x = origin_x - int(self.RadarRatio * npMaskedRanges[i] * math.cos(ang))
                    y = origin_y + int(self.RadarRatio * npMaskedRanges[i] * math.sin(ang))
                    print "i:%d ang:%f x:%d y:%d range:%f" %(i, np.rad2deg(ang),x,y,npMaskedRanges[i])
                elif i > 180 and i <= 270:
                    ang = np.deg2rad(270 - i)
                    x = origin_x + int(self.RadarRatio * npMaskedRanges[i] * math.cos(ang))
                    y = origin_y + int(self.RadarRatio * npMaskedRanges[i] * math.sin(ang))
                    print "i:%d ang:%f x:%d y:%d range:%f" %(i, np.rad2deg(ang),x,y,npMaskedRanges[i])
                elif i > 270 and i <= 359:
                    ang = np.deg2rad(i - 270)
                    x = origin_x + int(self.RadarRatio * npMaskedRanges[i] * math.cos(ang))
                    y = origin_y - int(self.RadarRatio * npMaskedRanges[i] * math.sin(ang))
                    print "i:%d ang:%f x:%d y:%d range:%f" %(i, np.rad2deg(ang),x,y,npMaskedRanges[i])
                #print "ang:%f x:%d y:%d" %(np.rad2deg(ang),x,y)
                radar.itemset((y,x,1),255)
        
        cv2.imshow('Radar',radar)
        cv2.waitKey(1)
        self.scanned.ranges = self.scan.ranges[:]
        return

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        #print self.scan.range_min
        #rospy.loginfo(self.scan)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        cv2.imshow("Image window", self.img)
        cv2.waitKey(1)

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
    bot = AllSensorBot(use_lidar=True, use_camera=False, use_imu=False,
                       use_odom=False, use_joint_states=False)
    bot.strategy()


