#!/usr/bin/env python2

import rospy
import tf
import math
from geometry_msgs.msg import Twist

from utils import Utils
from bearing import true_north_bearing

class Turtle_polaris(Utils):

    def __init__(self, *args):
        super(Turtle_polaris, self).__init__(*args)

    def obtain_yaw(self):
        q = (self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(q)

        return euler[2]

    def rotate(self, yaw_rad, yaw_req_rad):
        # simple P controller, a PI controller would be most appropriate but steady state error looks negligible
        yaw_e_rad = yaw_req_rad - yaw_rad

        if yaw_e_rad > math.pi:
            yaw_e_rad -= 2 * math.pi
        if yaw_e_rad < - math.pi:
            yaw_e_rad += 2 * math.pi

        if yaw_e_rad >= 0 :
            yaw_com_rad = min(self.max_angspeed_rad_s, self.kp * yaw_e_rad)
        else :
            yaw_com_rad = max(-float(self.max_angspeed_rad_s), self.kp * yaw_e_rad)

        rospy.loginfo("yaw error:"+str(yaw_e_rad))
        rospy.loginfo("yaw command:"+str(yaw_com_rad))

        # publish the command
        command = Twist()
        command.angular.z = yaw_com_rad
        self.cmd_vel_pub.publish(command)
    
    def run(self):
        rate = rospy.Rate(self.main_loop_freq)
        #note: this is defined outside of the execution loop as the robot is not moving location.
        yaw_req_rad = true_north_bearing(self.lat, self.lon)

        while not rospy.is_shutdown():
            yaw_rad = self.obtain_yaw()
            self.rotate(yaw_rad, yaw_req_rad)

            rospy.loginfo("current yaw:"+str(yaw_rad))
            rospy.loginfo("yaw req:"+str(yaw_req_rad))
            rate.sleep()

def main():
    rospy.init_node('turtle_polaris', anonymous=True)
    turtle = Turtle_polaris()
    turtle.run()

if __name__ == '__main__':
    main()

