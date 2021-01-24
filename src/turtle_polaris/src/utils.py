#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Subscribers, publishers, utility functions

class Utils(object):

    """Initialises the publisher, subscriber and loads the ros parameters."""

    def __init__(self, *args):

        # Subscriber
        self.odom = Odometry()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        #Parameters
        self.main_loop_freq = rospy.get_param('~loop_freq')
        self.max_angspeed_rad_s = rospy.get_param('~max_ang_speed_rad_s')
        self.kp = rospy.get_param('~kp')
        self.ki = rospy.get_param('~ki')
        self.lat = rospy.get_param('~lat')
        self.lon = rospy.get_param('~lon')
        self.alt_m = rospy.get_param('~alt')

        dict_topic_depends = { "/odom" : Odometry}
        success = self.wait_for_topics(dict_topic_depends) # wait for relevant subscribed topics to be ready
        if not success:
            raise rospy.ROSException("Cannot find topic dependencies.")

    def odom_callback(self, data):
        self.odom = data

    def wait_for_topics(self, dict_topics, timeout=30.0):
        """
        Wait for all required topics to be ready before proceeding.
        Inputs:
        - dict_topics   : dictionary of pairs: (topics : msg class)
        - timeout : float, maximum time spent waiting for topics (s)
        Returns:
        - bool_ready_to_proceed : (bool) whether or not topics are ready to proceed.
        """
        rospy.loginfo("Waiting for subscribed topics to start publishing...")

        bool_ready_to_proceed = False
        ntopics = len(dict_topics)

        t = 0.0
        dt = 0.1

        int_loop_freq = 1.0 / (dt*ntopics)  # Hz
        rate = rospy.Rate(int_loop_freq)

        dict_topics_ready = {topic_name : False for topic_name in dict_topics.keys()}
        keyval_topic_info = dict_topics.items()

        while t <= timeout and (not bool_ready_to_proceed): # timeout

            for topic_name, msg_class in keyval_topic_info:

                if not dict_topics_ready[topic_name]:
                    try:
                        rospy.wait_for_message(topic_name, msg_class, timeout=dt)
                        dict_topics_ready[topic_name] = True
                    except rospy.ROSException:
                        t += dt
                        pass

            array_topics_ready = dict_topics_ready.values()
            bool_ready_to_proceed = all(array_topics_ready)

            if (t % 3.0 == 0) and (not t == 0):
                rospy.loginfo("Waiting for topics, {0} of {1} found.".format( \
                    array_topics_ready.count(True), \
                    ntopics \
                ))

            rate.sleep()

        if bool_ready_to_proceed:
            rospy.loginfo("All ROS topics ready, time req.: {0} of {1}s".format( \
                t, timeout
            ))
        else:
            rospy.logwarn("ROS topics missing: timeout reached. {0} of {1} remaining.".format( \
                array_topics_ready.count(False), \
                ntopics \
            ))

        return bool_ready_to_proceed