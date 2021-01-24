#!/usr/bin/env python2

from ctypes import *
import os
import math

""" 
Wrapper file that contains functions to determine the True North Heading from 
computing the Magnetic Declination. The function computing the magnetic declination 
given a latitude, longitude and altitude is implemented in C, and is stored in the 
compiled library file declination.so. It is based on the World Magnetic Model 
2020-2025 and on open-source software released by NOAA (the National Oceanic and 
Atmospheric Administration).

References:
- Chulliat, Arnaud, et al. "The US/UK World Magnetic Model for 
2020-2025: Technical Report." (2020).

- WMM software: https://www.ngdc.noaa.gov/geomag/WMM/soft.shtml

"""

def declination(lat, lon, alt, date):
    file_path = os.path.dirname(os.path.abspath(__file__))
    b_file_path = file_path.encode('utf-8')
    so_file = file_path + '/declination.so'
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
