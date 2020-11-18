#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from rhd3d import HumanDetector3D
import numpy as np

# subscribe lidar msgs and change to numpy.ndarray
# publish centroid by pose array msgs from pointnet func 

# publish posearray 
# header -> frameid: map 
# time stamp -> sensormsg (lidar)
# quaterion x,y,z,w 2 -> object id


#pointcloud data to ndarray 
def callback(data):
    detector = HumanDetector3D(normalize_intensity=True)
    N = data.width #changed to pointcloud width

    #points = N x (x, y, z, intensity)
    points = np.zeros((N, 4)).astype(np.float32)
    results = detector.get_result(points)
    
    
def listener():
    rospy.init_node('pointcloud_listener', anonymous=True)
    rospy.Subscriber("/scan_matched_points2", PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
