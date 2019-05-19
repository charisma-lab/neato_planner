#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose

class NeatoTF(object):
    def __init__(self):
        rospy.init_node('neato_tf')
        self.br = tf.TransformBroadcaster()
        rospy.sleep(1.0)
        rospy.Subscriber('neato01/pose', Pose, self.pose_callback)
        self.transform_position = np.array([0., 0., 0.])
        self.transform_quaternion = np.array([0., 0., 0., 1.0])

    def pose_callback(self, pose):
        self.transform_position = pose.position
        self.transform_quaternion = pose.orientation

if __name__ == '__main__':
    try:
        td = NeatoTF()
        while not rospy.is_shutdown():
            td.br.sendTransform(td.transform_position,
                             td.transform_quaternion,
                             rospy.Time.now(),
                             "base_footprint",
                             "map")
            rospy.sleep(.1)
    except rospy.ROSInterruptException:
        pass
