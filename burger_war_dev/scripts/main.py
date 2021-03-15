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
import random
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
import time
import json
from std_msgs.msg import String

#LiDARの閾値
DETECT_POINT_NUM = 7

#SEARCHの初期化時間
SEARCH_INIT_TIME = 3
class MainState():
    """
    1. NAVIにてフィールドマーカを取得
    2. 取得後，SEARCHを行い敵を捜索
        2-1. 敵を見つけた場合
            ->相手がいる方向に回転（ATTACK）し，距離を走った(MOVE)後，手順2に遷移
        2-2. 敵を見つけてない場合（一度でもATTACKになったことがない）
            2-2-1. 手順1に遷移 　
        2-3. 敵を見つけてない場合（一度でもATTACKになったことがある）
            2-3-1. 得点が負けている場合は手順1に遷移
            2-3-1. 得点が勝っている場合は手順2に遷移
    """
    INIT        = 0    # 初期状態
    NAVI        = 1    # Navigationによる移動
    SEARCH      = 2    # 停止してLiDARによる検索
    ATTACK      = 3    # 相手方向に回転
    DEFFENCE    = 4    # Opencvによるディフェンス
    STOP        = 5    # 停止（degug）
    MOVE        = 6    # 相手方向に直進
    

class NaviTarget():
    """
    移動する先のフィールドのマーカをemunで定義している  
    """
    INIT            = 0
    LEFTLOWER_S     = 1    
    LEFTLOWER_N     = 2
    LEFTUPPER_S     = 3   
    LEFTUPPER_N     = 4        
    CENTER_N        = 5    
    CENTER_S        = 6    
    CENTER_E        = 7    
    CENTER_W        = 8    
    RIGHTLOWER_S    = 9    
    RIGHTLOWER_N    = 10
    RIGHTUPPER_S    = 11    
    RIGHTUPPER_N    = 12

class AllSensorBot(object):
    def __init__(self, 
                 use_lidar=False, use_camera=False, use_imu=False,
                 use_odom=False, use_joint_states=False):

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # State
        self.main_state      = MainState.INIT   # メイン状態
        self.prev_main_state = MainState.INIT   # 前回メイン状態
        self.next_state      = MainState.INIT   # 次状態

        # Navigation
        self.navi_target      = NaviTarget.INIT   # 目指すターゲット
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        #Score
        self.myColor     = None
        self.myScore     = 0
        self.enemyColor  = None
        self.enemyScore  = None
        self.warState    = None
        self.wartime     = 0
        self.score_sub   = rospy.Subscriber('/war_state', String, self.warStateCallback, queue_size=1)

        # lidar scan subscriber
        if use_lidar:
            self.scan = LaserScan()
            self.scanned = LaserScan()
            self.RadarRatio = 50
            self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)
            
            #LiDAR検知用グローバル変数
            self.npScanRanges = np.array(self.scan.ranges)
            self.npSubRanges = np.array(self.scan.ranges)

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

    ####################################
    #MOVINGで使用する関数
    ####################################
    # Navigation
    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
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
        else:
            return self.client.get_result()


    ####################################
     #SEARCHで使用する関数
    ####################################
    def is_detect_ememy_by_LiDAR(self):
        """
        LiDARで敵を検知したかを確認する
            ・閾値以上の距離が移動している点が３つ以上ある場合，検知したと判断する
            
        戻り値  True：検知/False：未検知
        """
        count = sum(self.npSubRanges)
        if count > DETECT_POINT_NUM:
            print("[Ture]Detect_EMEMY:%d " % count)
            return True
        else:
            print("[Flase]Detect_EMEMY:%d " % count)
            return False
    
    ####################################
     #得点管理
    ####################################
    #以下を参考にさせていただきました
    #https://raw.githubusercontent.com/rhc-ipponmanzoku/burger_war/master/burger_war/scripts/testRun.py
    def warStateCallback(self,data):
        warState = data
        jsonWarState = json.loads(warState.data)
        self.warState = jsonWarState["scores"]

         # which team?
        if jsonWarState["players"]["r"] == "you":
            self.myColor = "r"
            self.enemyColor = "b"
        else:
            self.myColor = "b"
            self.enemyColor = "r"

        #update myScore
        self.myScore = jsonWarState["scores"][self.myColor]

        #update enemyScore
        self.enemyScore = jsonWarState["scores"][self.enemyColor]

        #update war time
        self.wartime = jsonWarState["time"]
        
        print('=================================')
        print('myScore: {0}'.format(self.myScore))
        print('enemyScore: {0}'.format(self.enemyScore))
        print('wartime: {0}'.format(self.wartime))
        print('=================================')

    ####################################
     #状態処理関数
    ####################################
    def func_state_init(self):
        self.next_state = MainState.NAVI
        return

    def func_state_navigation(self):
        """
        1.  順番にマーカを取得しに行く
        """
        if self.navi_target == NaviTarget.INIT:           
            self.navi_target = NaviTarget.LEFTLOWER_S
            return

        if self.navi_target == NaviTarget.LEFTLOWER_S:           
            self.setGoal(-0.9,0.5,0)
            self.navi_target = NaviTarget.LEFTLOWER_N
            self.next_state = MainState.SEARCH
            return

        if self.navi_target == NaviTarget.LEFTLOWER_N:
            self.setGoal(-0.2,0.4,3.1415)
            self.navi_target = NaviTarget.CENTER_W
            self.next_state = MainState.SEARCH
            return

        if self.navi_target == NaviTarget.CENTER_W:
            self.setGoal(-0.2,0.4,-3.1415/2)
            self.navi_target = NaviTarget.LEFTUPPER_S
            self.next_state = MainState.SEARCH
            return

        if self.navi_target == NaviTarget.LEFTUPPER_S:  
            self.setGoal(-0.2,0.4,0)
            self.navi_target = NaviTarget.RIGHTLOWER_N
            self.next_state = MainState.SEARCH
            return

        if self.navi_target == NaviTarget.RIGHTLOWER_N:  
            self.setGoal(-0.2,-0.4,3.1415)
            self.navi_target = NaviTarget.CENTER_E
            self.next_state = MainState.SEARCH
            return

        if self.navi_target == NaviTarget.CENTER_E:  
            self.setGoal(-0.2,-0.4,3.1415/2)
            self.navi_target = NaviTarget.RIGHTUPPER_S
            self.next_state = MainState.SEARCH
            return

        if self.navi_target == NaviTarget.RIGHTUPPER_S:  
            self.setGoal(-0.2,-0.4,0)
            self.navi_target = NaviTarget.RIGHTLOWER_S
            self.next_state = MainState.SEARCH
            return

        if self.navi_target == NaviTarget.RIGHTLOWER_S:
            self.setGoal(-0.9,-0.5,0)
            #取得したいマーカをすべて獲得したので停止
            self.navi_target = NaviTarget.INIT
            self.next_state = MainState.SEARCH
            return
        
    def func_state_search(self):
        """
        1.  WAITを入れて移動していたときのLiDARの取得情報を初期化
        2.  閾値以上の距離が移動している点が３つ以上ある場合
                -> ATTACKモードに移行
            閾値以上の距離が移動している点が３つ以上ない場合
                -> NAVIモードに移行 or SEARCHモードに移行
        """
        
        time_count = 0

        #移動していたときのLiDARの取得情報を初期化
        while time_count <= SEARCH_INIT_TIME:
    
            # Radarの情報を取得
            if len(self.scan.ranges) != 0:
                bot.Radar()
            
            # 1秒Wait
            rospy.sleep(1)
            time_count = time_count + 1

        if self.is_detect_ememy_by_LiDAR():
            self.next_state = MainState.ATTACK
            return

        if self.prev_main_state == MainState.NAVI:
            self.next_state = MainState.NAVI
        else:
            if self.myScore > self.enemyScore:
                self.next_state = MainState.SEARCH
            else:
                self.next_state = MainState.NAVI

    def func_state_attack(self):
        
        #変数初期化
        npMaskedRanges = self.npScanRanges*self.npSubRanges
        total_angle = 0
        count = 0
    
        for i in range(len(npMaskedRanges)):
            if npMaskedRanges[i] != 0:    
                total_angle = total_angle + i
                count = count + 1
            if count >= DETECT_POINT_NUM:
                break

        #出現数をカウント
        #count = sum(self.npSubRanges)
        #敵がいる角度の計算
        angle_deg =  total_angle / count
        print("Ememy Position Angle(deg):%f " % angle_deg)
        
        angle_rad = np.deg2rad(angle_deg) 
        print("Ememy Position Angle(rad):%f " % angle_rad)
        
        print "**************"
        print "ATTACK!!!!!!!!!"
        print "**************"

        #旋回開始
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angle_rad
    
        # publish twist topic
        self.vel_pub.publish(twist)
        self.next_state = MainState.STOP
        return

    def func_state_defence(self):
        return

    def func_state_stop(self):
        print "**************"
        print "STOP!!!!!!!!!"
        print "**************"
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        # publish twist topic
        self.vel_pub.publish(twist)
        if self.prev_main_state == MainState.ATTACK:
            self.next_state = MainState.MOVE
        else:
            self.next_state = MainState.SEARCH
        return
    
    def func_state_moving(self):
        print "**************"
        print "MOVING!!!!!!!!!"
        print "**************"
        twist = Twist()
        twist.linear.x = 20; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        # publish twist topic
        self.vel_pub.publish(twist)

        self.next_state = MainState.STOP
        return

    def strategy(self):

        r = rospy.Rate(1)
     
        while not rospy.is_shutdown():
            # メイン状態処理を行う
            if self.main_state == MainState.INIT:
                # 初期化時
                self.func_state_init()
            elif self.main_state == MainState.NAVI:
                # 移動
                self.func_state_navigation()
            elif self.main_state == MainState.SEARCH:
                # 敵捜索
                self.func_state_search()
            elif self.main_state == MainState.ATTACK:
                # 敵に向けて回転
                self.func_state_attack()
            elif self.main_state == MainState.DEFFENCE:
                # 守り
                self.func_state_defence()
            elif self.main_state == MainState.STOP:
                # 停止
                self.func_state_stop()
            elif self.main_state == MainState.MOVE:
                # 敵に向けて全身
                self.func_state_moving()
            else:
                pass

            # DEBUG Print
            print('main_state = ',self.main_state)
            print('next_state = ',self.next_state)

            # メイン状態を次の状態に更新
            self.prev_main_state = self.main_state 
            self.main_state = self.next_state
            # 1秒Wait
            r.sleep()

    def Radar(self):
        """
        Radar map from LIDAR
        """
        print "Radar func"
        if len(self.scanned.ranges) == 0:
            self.scanned.ranges = self.scan.ranges[:]
        self.npScanRanges = np.array(self.scan.ranges)
        npScannedRanges = np.array(self.scanned.ranges)

        #LiDAR検知用グローバル変数に変更
        self.npSubRanges = abs(self.npScanRanges - npScannedRanges)
        for i in range(len(self.npSubRanges)):
            if self.npSubRanges[i] < 0.15:
                self.npSubRanges[i] = 0
            else:
                self.npSubRanges[i] = 1
        
        npMaskedRanges = self.npScanRanges*self.npSubRanges

        """
            if self.npSubRanges[i] != 0:
                print "i=%d Range=%f" %(i,self.npSubRanges[i])
        print self.npSubRanges
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
        
        """
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
        """
        #cv2.imshow('Radar',radar)
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


