#!/usr/bin/env python2
import unittest
import math
import rospy
import tf
from nav_msgs.msg import Odometry

#TODO : remove this
#import sys, os
#sys.path.append(os.path.dirname(os.path.abspath(__file__)))
#sys.path.append('../')

#from scripts.turtle_polaris_node import Turtle_polaris

PKG = 'turtle_polaris'

class Test_Turtle_polaris(unittest.TestCase):

    def setUp(self):

        #TODO add lon and lat params
        self.lat = rospy.get_param('~lat')
        self.lon = rospy.get_param('~lon')
        self.yaw_success_rad = rospy.get_param('~yaw_success')
        #TODO write several tests
        # 3 ways:
        # 1. find a way to terminate the turtle polaris controller node: not practical
        # 2. include the turtle polaris class in this file, calling the run function in a separate thread : not practical
        # 3. make different .test files: let's do this

        # Subscriber
        self.odom = Odometry()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, data):
        self.odom = data
    
    def obtain_yaw(self):
        q = (self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(q)

        return euler[2]

    def test_final_heading(self):

        acceptance = 0.01

        success = False
        while rospy.get_time() == 0.0:
            rospy.sleep(0.1)
        timeout = rospy.get_time() + 10.0

        while (not rospy.is_shutdown() and rospy.get_time() < timeout and(not success)):
            yaw_rad = self.obtain_yaw()
            yawrate_rad_s = self.odom.twist.twist.angular.z
            if abs(self.yaw_success_rad - yaw_rad) < acceptance and yawrate_rad_s < acceptance:
                success = True
            rospy.sleep(0.1)

        self.assert_(success)

        
if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)
    rostest.rosrun(PKG, 'test_turtle_polaris', Test_Turtle_polaris)