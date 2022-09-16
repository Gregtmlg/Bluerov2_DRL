import rospy
import subprocess
from os import path
from numpy import inf
import numpy as np
import random
import math
from gazebo_msgs.msg import ModelState
from squaternion import Quaternion
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import os
import time


class VelodyneNode():
    def listener(self):
        rospy.init_node("velodyne_reader")
        # self.setpoint_pub = rospy.Publisher("mavros/setpoint_motor/setpoint", MotorSetpoint, queue_size=1)
        self.velodyne = rospy.Subscriber('/velodyne_points', PointCloud2, self.velodyne_callback, queue_size=1)
        self.velodyne_data = np.ones(20) * 10
        self.gaps = [[-1.6, -1.57 + 3.14 / 20]]
        for m in range(19):
            self.gaps.append([self.gaps[m][1], self.gaps[m][1] + 3.14 / 20])
        self.gaps[-1][-1] += 0.03
        rospy.spin()

    # Read velodyne pointcloud and turn it into distance data, then select the minimum value for each angle
    # range as state representation
    def velodyne_callback(self, v):
        data = list(pc2.read_points(v, skip_nans=False, field_names=("x", "y", "z")))
        self.velodyne_data = np.ones(20) * 10
        for i in range(len(data)):
            if data[i][2] > -0.2:
                dot = data[i][0] * 1 + data[i][1] * 0
                mag1 = math.sqrt(math.pow(data[i][0], 2) + math.pow(data[i][1], 2))
                mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
                beta = math.acos(dot / (mag1 * mag2)) * np.sign(data[i][1])  # * -1
                dist = math.sqrt(data[i][0] ** 2 + data[i][1] ** 2 + data[i][2] ** 2)

                for j in range(len(self.gaps)):
                    if self.gaps[j][0] <= beta < self.gaps[j][1]:
                        self.velodyne_data[j] = min(self.velodyne_data[j], dist)
                        break
        rospy.loginfo(velodyne_data)



def main():
    node = VelodyneNode()
    node.listener()


if __name__ == "__main__":
    main()
