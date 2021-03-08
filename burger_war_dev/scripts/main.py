#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist

import tf


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

#for Get Score
import json
from std_msgs.msg import String

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


class NaviBot():
    def __init__(self):
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

 # score subscriver
        self.myColor     = None
        self.myScore     = 0
        self.enemyScore  = None
        self.warState    = None
        self.wartime     = 0
        self.score_sub   = rospy.Subscriber('/war_state', String, self.warStateCallback, queue_size=1)

        # StrategyState 
        self.StrategyState       = "Init"

    #以下を参考にさせていただきました
    #https://raw.githubusercontent.com/rhc-ipponmanzoku/burger_war/master/burger_war/scripts/testRun.py
    def warStateCallback(self,data):
        warState = data
        jsonWarState = json.loads(warState.data)
        self.warState = jsonWarState["scores"]

         # which team?
        if jsonWarState["players"]["r"] == "you":
            self.myColor = "r"
            self.enemyScore = "b"
        else:
            self.myColor = "b"
            self.enemyScore = "r"

        #update myScore
        self.myScore = jsonWarState["scores"][self.myColor]

        #update enemyScore
        self.enemyScore = jsonWarState["scores"][self.myColor]

        #update war time
        self.wartime = jsonWarState["time"]
        
        print('=================================')
        print('myScore: {0}'.format(self.myScore))
        print('enemyScore: {0}'.format(self.enemyScore))
        print('wartime: {0}'.format(self.wartime))
        print('=================================')
       
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
       
    def navi_rotate(self):

        self.setGoal(-0.5,0,0)
       

    def navi_defence(self):
        self.setGoal(-0.9,0,0)

    def check_state(self):
        if self.wartime  < 10:
            self.StrategyState = "Rotate"
        elif self.myScore >= self.enemyScore:
            self.StrategyState = "Defence"
        elif self.myScore < self.enemyScore:
            self.StrategyState = "Rotate"
        else:
            print "Error"
        #print('=================================')
        #print self.StrategyState
        #print('=================================')    

    
    def strategy(self):
    
        r = rospy.Rate(5) # change speed 5fps
        

        while not rospy.is_shutdown():
           
            self.check_state()

            if self.StrategyState == "Rotate":
                twist = self.navi_rotate()
            elif self.StrategyState == "Defence":
                twist = self.navi_defence()
            else:
                print "Error"
            print('=================================')
            print self.StrategyState
            print('=================================')    
            r.sleep()   



if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()
