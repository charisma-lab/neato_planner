#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
import math
import numpy as np
import tf.transformations as transform
from pure_pursuit_utils import *
from nav_msgs.msg import Path, Odometry
import copy


class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit_node')
        rospy.Subscriber('pose', PoseStamped, self.pf_pose_callback, queue_size=1)
        rospy.Subscriber('social_global_plan', Path, self.waypoints_list_cb, queue_size=1)
        self.drive_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.current_pose = [0, 0, 0]
        self.last_search_index = 0
        self.waypoints = []
        self.goal_lookahead_point = None
        self.waypoints_read_flag = False
        self.pose_read_flag = False

        self.LOOKAHEAD_DISTANCE = rospy.get_param('initial_lookahead_distance', 0.4)
        self.VELOCITY = rospy.get_param('initial_velocity', 0.3)
        self.SPEED_CONTROL = rospy.get_param('speed_control', False)
        self.STRAIGHT_LOOKAHEAD_DIST = rospy.get_param('straight_lookahead_dist', 3.0)
        self.TURNING_LOOKAHEAD_DIST = rospy.get_param('turning_lookahead_dist', 0.75)
        self.STRAIGHT_VEL = rospy.get_param('straight_vel', 2.0)
        self.TURNING_VEL = rospy.get_param('turning_vel', 0.75)
        self.TURN_WINDOW_MIN = rospy.get_param('turn_window_min', 3.75)
        self.TURN_WINDOW_MAX = rospy.get_param('turn_window_max', 4.5)
        self.MAX_X_DEVIATION = rospy.get_param('max_x_deviation', 4.75)
        self.RATE = rospy.get_param('controller_rate', 10)
        self.GOAL_THRESHOLD = rospy.get_param('goal_threshold', 0.15)

    def waypoints_list_cb(self, msg):
        if not self.waypoints_read_flag:
            print("\nWaypoints received:")
            self.waypoints = [(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) for pose in msg.poses]
            self.waypoints_read_flag = True

    def check_goal(self, goal):
        if dist(goal, self.current_pose) <= self.GOAL_THRESHOLD:
            rospy.loginfo("Goal reached")
            self.send_twist_vel(0.0, 0.0)
            return True
        else:
            return False

    def pf_pose_callback(self, msg):
        # 1. Determine the current location of the vehicle
        pose_x = msg.pose.position.x
        pose_y = msg.pose.position.y
        pose_yaw = quaternion_to_euler_yaw(msg.pose.orientation)
        self.current_pose = [pose_x, pose_y, pose_yaw]
        if not self.pose_read_flag:
            self.pose_read_flag = True
        # print "pose:=", pose_x, pose_y, pose_yaw

    def do_pure_pursuit(self):
        if self.waypoints_read_flag and self.pose_read_flag:
            last_index = len(self.waypoints) - 1
            if not self.check_goal(self.waypoints[last_index]):
                if self.SPEED_CONTROL:
                    average_x_from_car = self.get_x_deviation_in_path(self.current_pose[0], self.current_pose[1],
                                                                      self.current_pose[2])
                    self.VELOCITY = np.interp(average_x_from_car, [0.0, self.MAX_X_DEVIATION],
                                              [self.STRAIGHT_VEL, self.TURNING_VEL])
                    self.LOOKAHEAD_DISTANCE = np.interp(average_x_from_car, [0.0, self.MAX_X_DEVIATION],
                                                        [self.STRAIGHT_LOOKAHEAD_DIST, self.TURNING_LOOKAHEAD_DIST])

                # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.
                distance = 0.0
                for point in reversed(self.waypoints):
                    distance = dist(point, self.current_pose)
                    if distance <= self.LOOKAHEAD_DISTANCE:
                        self.goal_lookahead_point = point
                        break


                goal_point = self.goal_lookahead_point

                # 3. Transform the goal point to vehicle coordinates.
                point_x_wrt_car = self.compute_x_wrt_car(goal_point[0] - self.current_pose[0],
                                                         goal_point[1] - self.current_pose[1], self.current_pose[2],
                                                         distance)

                # 4. Calculate the curvature = 1/r = 2x/l^2
                # angle = -2.0 * point_x_wrt_car / distance ** 2
                if distance != 0:
                    radius = distance ** 2 / (-2.0 * point_x_wrt_car)
                    angular_velocity = self.VELOCITY / radius
                    angular_velocity = np.clip(angular_velocity, -2, 2)
                    self.send_twist_vel(self.VELOCITY, angular_velocity)
                elif distance == 0:
                    self.send_twist_vel(0,0)

            self.waypoints_read_flag = True

    def send_twist_vel(self, linear_x, angular_z):
        twist_message = Twist()
        twist_message.linear.x = linear_x
        twist_message.angular.z = angular_z
        self.drive_pub.publish(twist_message)

    def get_x_deviation_in_path(self, pose_x, pose_y, yaw):
        average_x_from_car = 0.0
        window_points_count = 0

        for index, point in enumerate(self.waypoints):
            if index < self.last_search_index:
                continue
            distance = dist(point, self.current_pose)
            if self.TURN_WINDOW_MIN <= distance <= self.TURN_WINDOW_MAX:
                average_x_from_car += self.compute_x_wrt_car(point[0] - pose_x, point[1] - pose_y, yaw, distance)
                window_points_count += 1
            if distance > self.TURN_WINDOW_MAX:
                break
        if window_points_count == 0:
            return 0.0
        average_x_from_car = average_x_from_car / window_points_count
        average_x_from_car = np.clip(abs(average_x_from_car), 0.0, self.MAX_X_DEVIATION)
        return average_x_from_car

    def compute_x_wrt_car(self, deltax, deltay, yaw, distance):
        beta = math.atan2(deltax, deltay)
        gamma = math.pi / 2 - yaw - beta
        x_wrt_car = -1.0 * distance * math.sin(gamma)
        return x_wrt_car

    def run_pure_pursuit(self):
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            self.do_pure_pursuit()
            rate.sleep()


if __name__ == '__main__':
    pure_pursuit = PurePursuit()
    pure_pursuit.run_pure_pursuit()
    rospy.spin()
