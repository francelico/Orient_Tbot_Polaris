#!/usr/bin/env python2

import rospy

class PI(object):

    def __init__(self, P=1.0, I=0.0):

        self.kp = P
        self.ki = I
        self.current_time = rospy.get_time()
        self.previous_time = self.current_time

        self.clear()

    def clear(self):
        self.P_term = 0.0
        self.I_term = 0.0

    def update(self, error):
        self.current_time = rospy.get_time()
        self.P_term = self.kp * error
        self.I_term += self.ki * error * (self.current_time - self.previous_time)
        u = self.P_term + self.I_term
        self.previous_time = self.current_time
        return u