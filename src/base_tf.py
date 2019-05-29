#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
import tf_conversions.posemath as posemath

class MCLTf(object):
    def __init__(self):
        rospy.init_node('neato_tf')
        self.br = tf.TransformBroadcaster()
        self.tf_listener =  tf.TransformListener()
        rospy.sleep(1.0)
        rospy.Subscriber('/pose', PoseStamped, self.pose_callback)
        self.pose_initialized= True
        self.transform_position = np.array([1., 1., 0.])
        self.transform_quaternion = np.array([0., 0., 0., 1.0])

    def pose_callback(self, pose):
        try:
            self.tf_listener.waitForTransform('map',
                                              'odom',
                                              pose.header.stamp, 
                                              rospy.Duration(1.0))
            frame = posemath.fromMsg(pose.pose).Inverse()
            pose.pose = posemath.toMsg(frame)
            pose.header.frame_id = 'base_footprint'
            odom_pose = self.tf_listener.transformPose('odom',pose)
            frame = posemath.fromMsg(odom_pose.pose).Inverse()
            odom_pose.pose = posemath.toMsg(frame)

            self.transform_position[0] = odom_pose.pose.position.x
            self.transform_position[1] = odom_pose.pose.position.y
            self.transform_quaternion[0] = odom_pose.pose.orientation.x
            self.transform_quaternion[1] = odom_pose.pose.orientation.y
            self.transform_quaternion[2] = odom_pose.pose.orientation.z
            self.transform_quaternion[3] = odom_pose.pose.orientation.w
            if not self.pose_initialized:
                self.pose_initialized = True
        except tf.Exception as e:
            print('EXCEPTION - ', e)

if __name__ == '__main__':
    try:
        td = MCLTf()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if td.pose_initialized:
                td.br.sendTransform(td.transform_position,
                                td.transform_quaternion,
                                rospy.Time.now(),
                                "odom",
                                "map")
                rate.sleep()
    except rospy.ROSInterruptException:
        pass
