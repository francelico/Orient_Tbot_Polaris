#!/usr/bin/env python2

from ctypes import *
import os
import math

def declination(lat, lon, alt, date):
    file_path = os.path.dirname(os.path.abspath(__file__))
    b_file_path = file_path.encode('utf-8')
    so_file = file_path + '/declination_test.so'
    lib = CDLL(so_file)
    lib.declination.argtypes = [c_double, c_double, c_double, c_double, c_char_p]
    lib.declination.restype = c_float

    alt *= 1e-3 #convert to km
    return lib.declination(lat , lon , alt , date, b_file_path)

def true_north_heading(lat, lon, alt):
    # This function assumes the year to be 2021
    YEAR = 2021
    YAW_NORTH_RAD = math.pi / 2 # to respect ENU reference frame of ROS

    dec = declination(lat, lon, alt, YEAR)
    true_north_heading_rad = math.radians(dec) + YAW_NORTH_RAD
    return true_north_heading_rad
