#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is rumdom run node.
subscribe No topcs.
Publish 'cmd_vel' topic. 
mainly use for simple sample program

by Takuya Yamaguhi.
'''

import rospy
import random

from geometry_msgs.msg import Twist


class RandomBot():
    def __init__(self, bot_name="NoName"):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

    def calcTwist(self, x, th):
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        r.sleep()
        r.sleep()
        twist = self.calcTwist(-0.3,0)
        print(twist)
        self.vel_pub.publish(twist)
        r.sleep()
        twist = self.calcTwist(0,-0.75)
        print(twist)
        self.vel_pub.publish(twist)
        r.sleep()
        twist = self.calcTwist(1, 0)
        print(twist)
        self.vel_pub.publish(twist)
        r.sleep()

        while not rospy.is_shutdown():
            # 
            for counter in range(9):
                twist = self.calcTwist(10, 0)
                print(twist)
                self.vel_pub.publish(twist)
                r.sleep()

            # back
            twist = self.calcTwist(-0.4, 0)
            print(twist)
            self.vel_pub.publish(twist)
            r.sleep()

            # turn
            twist = self.calcTwist(0, 1.7)
            print(twist)
            self.vel_pub.publish(twist)
            r.sleep()
 


if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random')
    bot.strategy()

