#!/usr/bin/env python

'''
This node will generate the higher level trajectory for different behaviors such as "happy", Grumpy", and "Sleepy".
'''

import rospy
import math
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Pose, Point, Quaternion
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler


def create_header(frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return header


def quat_heading(yaw):
    q = quaternion_from_euler(0, 0, yaw)
    return Quaternion(*q)


class BehaviorGenerator:
    def __init__(self):
        self.waypoint_publisher = rospy.Publisher('/waypoints_list', Path, queue_size=1)
        rospy.Subscriber('/goal', Pose, self.goal_callback, queue_size=1) # Use of service could be more efficient
        rospy.Subscriber('/start', Pose, self.start_callback, queue_size=1)
        rospy.Subscriber('/obstacle', Pose, self.obstacle_callback, queue_size=1)
        self.waypoints_pub = rospy.Publisher('/waypoints_list', Path, queue_size=1)

        self.start = None
        self.goal = None
        self.obstacle = None

    def goal_callback(self, goal_msg):
        # extract the pose and update
        self.goal = [goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y]

    def start_callback(self, start_msg):
        self.start = [start_msg.pose.pose.position.x, start_msg.pose.pose.position.y]

    def obstacle_callback(self, obstacle_msg):
        self.obstacle = obstacle_msg

    def populate_path_msg(self, rotated_path, heading):
        poses_waypoints = []

        for i, point in rotated_path:
            header = create_header('map')
            waypoint = Pose(Point(float(point[0]), float(point[1]), 0), quat_heading(float(heading[i])))
            poses_waypoints.append(PoseStamped(header, waypoint))
        return poses_waypoints

    def gen_trajectory(self, behavior, amplitude=1.0):
        if behavior == 1:
            # generate sinusoidal path
            y_diff = self.goal[1] - self.start[1]
            x_diff = self.goal[0] - self.start[0]
            theta = math.atan2(y_diff, x_diff)
            dist = math.sqrt(x_diff**2 + y_diff**2)
            freq = 0.2
            dist_int = int(np.clip(dist, 0, dist))  # Does this make sense?

            path = np.linspace(0, dist_int, num=100)
            omega = 2*np.pi*freq
            path = np.array([[L, amplitude*np.sin(omega*L)] for L in path])
            # Now we have a path, oscillating on the x-axis

            rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

            path_rotated = np.zeros([path.shape[0], path.shape[1]])
            heading = np.zeros([path.shape[0],1])

            for i, point in enumerate(path):
                result = np.matmul(rot, point)
                path_rotated[i] = result
                # Now, path is from start point to goal point

            # Find heading for each point
            for i in range(len(path_rotated)-1):
                dx = path_rotated[i+1, 0] - path_rotated[i, 0]
                dy = path_rotated[i+1, 1] - path_rotated[i, 1]
                heading[i] = math.atan2(dy, dx)
            heading[-1] = heading[-2]

            # Now, populate the Path message
            path_to_publish = Path()
            path_to_publish.header = create_header('map')
            path_to_publish.poses = self.populate_path_msg(path_rotated, heading)
            self.waypoint_publisher.publish(path_to_publish)

        if behavior == 2:
            # generate zig-zag, non-smooth path
            pass

        if behavior == 3:
            # generate sinusoidal with less frequency?
            pass


if __name__ == "__main__":
    # 1: happy, #2: grumpy, #3: sleepy
    behavior = 1
    generator = BehaviorGenerator()
    generator.gen_trajectory(behavior)
