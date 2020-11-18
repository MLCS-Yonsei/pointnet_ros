#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from rhd3d import HumanDetector3D
import numpy as np

import pcl
import pcl_helper

# subscribe lidar msgs and change to numpy.ndarray
# publish centroid by pose array msgs from pointnet func 

# publish posearray 
# header -> frameid: map 
# time stamp -> sensormsg (lidar)
# quaterion x,y,z,w 2 -> object id


class PointnetROS:

    def __init__(self):

        rospy.init_node('pointcloud_listener', anonymous=True)

        posearray_topic = rospy.get_param("posearray_topic")
        self.pub = rospy.Publisher(posearray_topic, PoseArray, queue_size=10)

        rospy.Subscriber("/scan_matched_points2", PointCloud2, self.callback)

        self.detector = HumanDetector3D(normalize_intensity=True)

        rospy.spin()


    def callback(self, data):
        #pointcloud data to ndarray 
        N = data.width #changed to pointcloud width

        #points = N x (x, y, z, intensity)
        points = pcl_helper.ros_to_pcl(data)

        results = self.detector.get_result(points)

        msg = PoseArray()
        msg.header = rospy.Time.now()
        for object_id in range(results.shape[0]):
            pose = Pose()
            pose.orientation.w = object_id
            pose.orientation.x = results[object_id, 0]
            pose.orientation.y = results[object_id, 1]
            pose.orientation.z = results[object_id, 2]
            msg.poses.append(pose)

        self.pub.publish(msg)



if __name__ == '__main__':

    pointnet = PointnetROS()
