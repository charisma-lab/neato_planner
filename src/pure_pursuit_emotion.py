#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
import math
import numpy as np
from pure_pursuit_utils import *
from nav_msgs.msg import Path, Odometry
import copy
import time, sys

class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit_node')
        self.emotion_pub = rospy.Publisher('emotional_state', String, queue_size=1)
        self.drive_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.current_pose = [0, 0, 0]
        self.last_search_index = 0
        self.waypoints = []
        self.goal_lookahead_point = None
        self.waypoints_read_flag = False
        self.changed_goal = False
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
        self.GOAL_THRESHOLD = rospy.get_param('goal_threshold', 0.5)
        self.INTERMEDIATE_GOAL_THRESHOLD=rospy.get_param('intermediate_goal_threshold', 0.15)
        self.EXECUTION_DELAY_MIN = rospy.get_param('execution_delay_min', 3.0)
        self.EXECUTION_DELAY_MAX = rospy.get_param('execution_delay_max', 6.0)
        self.VEL_MIN = rospy.get_param('sleepy_vel_min', 0.15)
        self.VEL_MAX = rospy.get_param('sleepy_vel_max', 0.6)
        self.target_velocity = self.VELOCITY
        self.previous_velocity = 0.0
        self.EXECUTION_DELAY = self.random_sample(self.EXECUTION_DELAY_MIN, self.EXECUTION_DELAY_MAX)
        self.last_time = time.time()
        self.RAMP_RATE = rospy.get_param('ramp_rate', 0.25)  #m/s^2
        self.CURRENT_STATE = rospy.get_param('emotional_state', "happy")

        self.ROTATION_TOLERANCE = 0.15
        self.angular_velocity_grumpy = math.radians(30)
        self.current_index = 0

        rospy.Subscriber('pose', PoseStamped, self.pf_pose_callback, queue_size=1)
        rospy.Subscriber('social_global_plan', Path, self.waypoints_list_cb, queue_size=1)
        rospy.Subscriber('social_planning_mode', String, self.social_planning_mode_callback, queue_size=1)
        rospy.Subscriber('changed_goal', Bool, self.changed_goal_callback, queue_size=1)

    def waypoints_list_cb(self, msg):
        if (not self.waypoints_read_flag) or self.changed_goal :
            print("\nWaypoints received:")
            self.waypoints = [(pose.pose.position.x, pose.pose.position.y, quaternion_to_euler_yaw(pose.pose.orientation)) for pose in msg.poses]
            self.waypoints_read_flag = True
            self.current_index=0
            self.changed_goal=False

    def check_goal(self, goal,threshold):
        if dist(goal, self.current_pose) <= threshold:
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
            if not self.check_goal(self.waypoints[last_index],self.GOAL_THRESHOLD):
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
                    if self.CURRENT_STATE == "happy":
                        self.send_twist_vel(self.VELOCITY, angular_velocity)
                    elif self.CURRENT_STATE == "sleepy":
                        self.random_sample_velocity_delay(angular_velocity)
                elif distance == 0:
                    self.send_twist_vel(0, 0)

            self.waypoints_read_flag = False

    def send_twist_vel(self, linear_x, angular_z):
        twist_message = Twist()
        twist_message.linear.x = linear_x
        twist_message.angular.z = angular_z
        self.drive_pub.publish(twist_message)

    def random_sample(self,low, high):
        return np.random.random() * (high - low) + low

    def social_planning_mode_callback(self, msg):
        self.CURRENT_STATE = msg.data
        print "current state=", self.CURRENT_STATE

    def random_sample_velocity_delay(self, angular_z):
        present_time = time.time()
        if (present_time - self.last_time) >= self.EXECUTION_DELAY:
            self.EXECUTION_DELAY = self.random_sample(self.EXECUTION_DELAY_MIN, self.EXECUTION_DELAY_MAX)
            self.previous_velocity = copy.deepcopy(self.VELOCITY)
            self.VELOCITY = self.random_sample(self.VEL_MIN, self.VEL_MAX)
            # angular_z = angular_z + self.random_sample(-0.3,0.3)
            delay = self.random_sample(self.EXECUTION_DELAY_MIN/2, self.EXECUTION_DELAY_MAX/2)
            self.last_time = present_time
        else:
            delay = 0.0
        if delay > 0:
            self.take_a_stop(angular_z)
            sleep_duration = rospy.Duration(delay, 0)
            rospy.sleep(sleep_duration)
        self.send_twist_vel(self.VELOCITY, angular_z)

    def take_a_stop(self, angular_z):
        rate_stop = rospy.Rate(self.RATE)
        while self.previous_velocity != 0:
            linear_velocity = self.ramp_velocity(0.0, self.previous_velocity, self.RAMP_RATE)
            if linear_velocity == 0.0:
                self.send_twist_vel(linear_velocity, 0.0)
            else:
                self.send_twist_vel(linear_velocity, angular_z/2.0)
            self.previous_velocity = linear_velocity
            rate_stop.sleep()

    def ramp_velocity(self, target_vel, previous_vel, ramp_rate):
        sign = 1 if target_vel >= previous_vel else -1
        if sign == 1:
            return target_vel
        step_size = ramp_rate / self.RATE
        delta = math.fabs(target_vel - previous_vel)
        if delta >= step_size:
            command_vel = previous_vel + sign * step_size
        else:
            command_vel = target_vel
        return command_vel

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

    def changed_goal_callback(self,msg):
        self.changed_goal = msg.data

    def rotate(self,angle):
        print("angle is : {} and heading is : {}".format(angle, self.current_pose[2]))
        while abs(angle-self.current_pose[2])>self.ROTATION_TOLERANCE:
            if (angle-self.current_pose[2])>0:
                sign=1
            else:
                sign=-1
            self.send_twist_vel(0,sign*self.angular_velocity_grumpy)
            rospy.sleep(0.1)
            print("angle is : {} and heading is : {}".format(angle, self.current_pose[2]),sign)

        # self.send_twist_vel(0, 0)

    def do_grumpy_navigation(self):
        self.current_index=0 #assign this zero everytime we get new set of waypoints
        delta_y=self.waypoints[self.current_index][1]-self.current_pose[1]
        delta_x=self.waypoints[self.current_index][0]-self.current_pose[0]
        heading = math.atan2(delta_y,delta_x)
        while (not rospy.is_shutdown()) or (self.current_index!=(len(self.waypoints)-1)):
            self.rotate(heading)
            if self.current_index == (len(self.waypoints)-2):
                if self.check_goal(self.waypoints[self.current_index+1],self.GOAL_THRESHOLD):
                    self.current_index +=1
                    delta_y=self.waypoints[self.current_index][1]-self.current_pose[1]
                    delta_x=self.waypoints[self.current_index][0]-self.current_pose[0]
                    heading = math.atan2(delta_y,delta_x)
            else:
                if self.check_goal(self.waypoints[self.current_index+1],self.INTERMEDIATE_GOAL_THRESHOLD):
                    self.current_index +=1
                    delta_y=self.waypoints[self.current_index][1]-self.current_pose[1]
                    delta_x=self.waypoints[self.current_index][0]-self.current_pose[0]
                    heading = math.atan2(delta_y,delta_x)
            self.send_twist_vel(self.VELOCITY,0.0)
        self.rotate(self.waypoints[self.current_index-1])
        self.send_twist_vel(0.0,0.0)

    def run_pure_pursuit(self):
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            if self.CURRENT_STATE == "grumpy":
                self.do_grumpy_navigation()
            else:
                self.do_pure_pursuit()
            self.emotion_pub.publish(String(self.CURRENT_STATE))
            rate.sleep()


if __name__ == '__main__':
    pure_pursuit = PurePursuit()
    key_input = input('Enter my emotion: \n 1: happy \t 2: grumpy \t 3: sleepy \n')
    if key_input == 1:
	    pure_pursuit.CURRENT_STATE = "happy"
    elif key_input == 2:
	    pure_pursuit.CURRENT_STATE = "grumpy"
    elif key_input == 3:
	    pure_pursuit.CURRENT_STATE = "sleepy"
    rospy.sleep(5.0)
    pure_pursuit.run_pure_pursuit()
    rospy.spin()
