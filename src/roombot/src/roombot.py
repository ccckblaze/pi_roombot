#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import logging
from controller import Viehcle

class Roombot(object):

    def __init__(self):
        logging.basicConfig(level=logging.DEBUG)
        logging.debug('Logger ready!')
 
        rospy.init_node('roombot', log_level=rospy.DEBUG)
  
        global viehcle
        viehcle = Viehcle(False, 0)
       
        rospy.Subscriber('/roombot/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.spin()
        
        viehcle.destroy()
        del viehcle
        
    def cmd_vel_callback(self, msg):
        rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
        viehcle.setSpeed(msg.linear.x) // test
        viehcle.forward()
