#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def callback(data):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    # fill in data from your vicon message
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"  # Assuming "world" is your fixed frame
    t.child_frame_id = "/vicon/frame/frame"  # Assuming "base_link" is the root of your URDF
    t.transform = data.transform  # Copy transform from vicon/frame

    br.sendTransform(t)

def main():
    rospy.init_node('vicon_to_tf_broadcaster')
    rospy.Subscriber('vicon/frame/frame', TransformStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
