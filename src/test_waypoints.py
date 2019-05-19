#!/usr/bin/env python

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rospy
import tf

'''
This is a test program for publishing target pose to move_base action server.
Currently, it is working with one waypoints, and way breaking when testing with multiple waypoints.
The problem might be in the execution code, waypoint_execution.
'''

if __name__=="__main__":
    # Initialize node
    rospy.init_node("test_waypoint_publisher")

    particles_pub = rospy.Publisher('/waypoints_list', Path, queue_size=1)

    p1 = PoseStamped()

    p1.pose.position.x = 5.0
    p1.pose.position.y = 5.0

    yaw = 0.0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    p1.pose.orientation.x = quaternion[0]
    p1.pose.orientation.y = quaternion[1]
    p1.pose.orientation.z = quaternion[2]
    p1.pose.orientation.w = quaternion[3]

    waypoints = Path()
    waypoints.poses.append(p1)

    flag = True 

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if flag:
            print("Wait 10 seconds for things to load, before publishing waypoints...")
            rospy.sleep(10.0)
            particles_pub.publish(waypoints)
            flag = False
        rate.sleep()

    print("Waypoints published.")
