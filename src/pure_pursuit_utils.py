#!/usr/bin/env python

import rospy
import math
import numpy as np
import tf.transformations as transform
import csv
import os


def read_waypoints_from_csv(filename):
    # Import waypoints.csv into a list (path_points)
    if filename == '':
        raise ValueError('No any file path for waypoints file')
    with open(filename) as f:
        path_points = [tuple(line) for line in csv.reader(f, delimiter=',')]
    path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]
    return path_points


def dist(p1, p2):
    """Calculate sthe straight line distance between two points (p1 and p2)"""
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) # ** is exponent 


# Computes euler yaw angle from the quaternion data
def quaternion_to_euler_yaw(orientation):
    _, _, yaw = transform.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
    return yaw


# Constrain a given value between lower and upper bounds
def constrain(value, min, max):
    if value > max:
        value = max
    elif value < min:
        value = min
    return value
