#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Subscribers, publishers, utility functions

class Utils(object):

    def __init__(self, *args):

        self.odom = Odometry()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        #TODO add a wait for topic function

        #dict_topic_depends = { "/odom" : Odometry}
        # success = wait_for_topics(dict_topic_depends) # wait for relevant subscribed topics to be ready
        # if not success:
        #     raise rospy.ROSException("Cannot find topic dependencies.")


        #TODO add loop_freq as a param
        self.main_loop_freq = 30 #(rosparam)

    def odom_callback(self, data):
        self.odom = data