#!/usr/bin/env python3

import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

def ply_to_pointcloud2():
    rospy.init_node('ply_to_pointcloud', anonymous=True)
    pub = rospy.Publisher('pointcloud_output', PointCloud2, queue_size=10)

    # Load PLY into PointCloud
    cloud = pcl.load("/home/kris/catkin_ws/src/my_package/meshes/frame.ply")
    
    # Convert the PCL PointCloud to a list of points
    cloud_points = cloud.to_list()

    # Create a ROS PointCloud2 message
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "vicon/frame/frame"  # or whatever frame_id you want to use
    output = pc2.create_cloud_xyz32(header, cloud_points)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        pub.publish(output)
        rate.sleep()

if __name__ == '__main__':
    try:
        ply_to_pointcloud2()
    except rospy.ROSInterruptException:
        pass
