#!/usr/bin/env python
import rospy

# https://hotblackrobotics.github.io/en/blog/2018/01/29/seq-goals-py/

import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path


class MoveBaseSeq():

    def __init__(self):

        #Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...will wait 120 seconds")
        self.client.wait_for_server(rospy.Duration(120.0))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

        #self.movebase_client()
        self.waypoints_list = []
        self.goal_cnt = 0

        rospy.Subscriber('/waypoints_list', Path, self.waypoints_list_cb, queue_size=1)
        print("Subscriber waiting for waypoints_list...")

        self.flag = True


    def waypoints_list_cb(self, msg):
        if self.flag:
            self.waypoints_list = msg
            print("\nWaypoints received:")
            for pose in self.waypoints_list.poses:
                print([pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.z])
            self.movebase_client()
            print("Received waypoints!")
            self.flag = False

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt< len(self.waypoints_list.poses):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.waypoints_list.poses[self.goal_cnt].pose
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.waypoints_list.poses[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                rospy.sleep(5.0)
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.waypoints_list.poses[self.goal_cnt].pose
        print(goal.target_pose.pose.position.x)
        print(goal.target_pose.pose.position.y)

        # TODO: convert orientation to euler and print if necessary
        
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.waypoints_list.poses[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('execute_waypoints')
    rate = rospy.Rate(5)
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

    while not rospy.is_shutdown():
        rate.sleep()
