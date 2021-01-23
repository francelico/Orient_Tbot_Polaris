#!/usr/bin/env python2

import math

def true_north_bearing(lat, lon):
    # North Magnetic dip pole latitude and longitude
    # Note: MAG_N_LAT and MAG_N_LON should be updated every year according to the International Geomagnetic Reference Field
    MAG_N_LAT = 86.83
    MAG_N_LON = 164.07
    YAW_NORTH_RAD = math.pi / 2 # to respect ENU reference frame of ROS

    mag_n_lat_rad = math.radians(MAG_N_LAT)
    mag_n_lon_rad = math.radians(MAG_N_LON)

    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    y = math.sin(mag_n_lon_rad - lon_rad) * math.cos(mag_n_lat_rad)
    x = math.cos(lat_rad) * math.sin(mag_n_lat_rad) - math.sin(lat_rad) * math.cos(mag_n_lat_rad) * math.cos(mag_n_lon_rad - lon_rad)

    bearing_rad = YAW_NORTH_RAD + math.atan2(y,x)
    return bearing_rad
