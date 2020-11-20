#!/usr/bin/env python3
import rospy
import ros_numpy

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from rhd3d import HumanDetector3D
import numpy as np

# import python_pcl
# import pcl_helper

# subscribe lidar msgs and change to numpy.ndarray
# publish centroid by pose array msgs from pointnet func 

# publish posearray 
# header -> frameid: map 
# time stamp -> sensormsg (lidar)
# quaterion x,y,z,w 2 -> object id


class PointnetROS:

    def __init__(self):

        rospy.init_node('pointcloud_listener', anonymous=True)

        posearray_topic = rospy.get_param("posearray_topic", "/people_pose")
        self.pub = rospy.Publisher(posearray_topic, PoseArray, queue_size=10)

        rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)

        self.detector = HumanDetector3D(normalize_intensity=True)

        rospy.spin()


    def callback(self, data):
        #pointcloud data to ndarray [ N x (x, y, z, intensity) ]
        N = data.width #changed to pointcloud width
        pc = ros_numpy.numpify(data)
        points = np.zeros((N, 4)).astype(np.float32)
        points[:,0] = pc['x']
        points[:,1] = pc['y']
        points[:,2] = pc['z']
        points[:,3] = pc['intensity']

        # N = data.width #changed to pointcloud width
        # points = np.zeros((N, 4)).astype(np.float32)
        # i=0
        # for p in pc2.read_points(data, field_names=("x", "y", "z","intensity")):
        #     point = [p[0],p[1],p[2],p[3]]
        #     points[i] = point
        #     i=i+1

        #get result of pointnet
        results = self.detector.get_result(points)
        
        #publish by posearray msg
        msg = PoseArray()
        msg.header.frame_id = "velodyne"
        msg.header.stamp = rospy.Time.now()

        for idx in range(results.shape[0]):
            pose = Pose()
            pose.position.x = results[idx,0]
            pose.position.y = results[idx,1]
            pose.position.z = results[idx,2]
            pose.orientation.w = results[idx, 3]
            msg.poses.append(pose)

        self.pub.publish(msg)



if __name__ == '__main__':

    pointnet = PointnetROS()
