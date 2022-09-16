import rospy
import subprocess
from os import path
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from numpy import inf
import numpy as np
import random
import math
from gazebo_msgs.msg import ModelState
from squaternion import Quaternion
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import os
import time

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'


# Check if the random goal position is located on an obstacle and do not accept it if it is
def check_pos(x, y):
    goalOK = True

    if -3.8 > x > -6.2 and 6.2 > y > 3.8:
        goalOK = False

    if -1.3 > x > -2.7 and 4.7 > y > -0.2:
        goalOK = False

    if -0.3 > x > -4.2 and 2.7 > y > 1.3:
        goalOK = False

    if -0.8 > x > -4.2 and -2.3 > y > -4.2:
        goalOK = False

    if -1.3 > x > -3.7 and -0.8 > y > -2.7:
        goalOK = False

    if 4.2 > x > 0.8 and -1.8 > y > -3.2:
        goalOK = False

    if 4 > x > 2.5 and 0.7 > y > -3.2:
        goalOK = False

    if 6.2 > x > 3.8 and -3.3 > y > -4.2:
        goalOK = False

    if 4.2 > x > 1.3 and 3.7 > y > 1.5:
        goalOK = False

    if -3.0 > x > -7.2 and 0.5 > y > -1.5:
        goalOK = False

    if x > 4.5 or x < -4.5 or y > 4.5 or y < -4.5:
        goalOK = False
    return goalOK


# Function to put the laser data in bins
def binning(lower_bound, data, quantity):
    width = round(len(data) / quantity)
    quantity -= 1
    bins = []
    for low in range(lower_bound, lower_bound + quantity * width + 1, width):
        bins.append(min(data[low:low + width]))
    return np.array([bins])


class GazeboEnv:
    """Superclass for all Gazebo environments.
    """
    metadata = {'render.modes': ['human']}

    def __init__(self, launchfile, height, width, nchannels):

        self.odomX = 0
        self.odomY = 0

        self.goalX = 1
        self.goalY = 0.0

        self.upper = 5.0
        self.lower = -5.0
        self.velodyne_data = np.ones(20) * 10
        self.last_laser = None
        self.last_odom = None

        # Construction variables thrusters
        self.thrust = 0.0
        self.thrust_stepsize = 0.1
        self.thrust_scaler = 0.6 #0.4
        self.lateral_thrust = 0.0
        self.lateral_thrust_stepsize = 0.1
        self.lateral_thrust_scaler = 0.4
        self.vertical_thrust = 0.0
        self.vertical_thrust_stepsize = 0.1
        self.vertical_thrust_scaler = 0.4
        self.yaw_rate = 0.0
        self.yaw_rate_stepsize = 0.1
        self.yaw_rate_scaler = 0.1 #0.2


        self.set_self_state = ModelState()
        self.set_self_state.model_name = 'uuv_bluerov2_heavy_0'
        self.set_self_state.pose.position.x = 0.
        self.set_self_state.pose.position.y = 0.
        self.set_self_state.pose.position.z = 0.
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0
        self.distOld = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))
        self.gaps = [[-1.6, -1.57 + 3.14 / 20]]
        for m in range(19):
            self.gaps.append([self.gaps[m][1], self.gaps[m][1] + 3.14 / 20])
        self.gaps[-1][-1] += 0.03

        port = '11311'
        subprocess.Popen(["roscore", "-p", port])

        print("Roscore launched!")

        # Launch the simulation with the given launchfile name
        rospy.init_node('TD3')
        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), "assets", launchfile)
        if not path.exists(fullpath):
            raise IOError("File " + fullpath + " does not exist")

        subprocess.Popen(["roslaunch", "-p", port, fullpath])
        print("Gazebo launched!")

        self.gzclient_pid = 0

        # Set up the ROS publishers and subscribers
        self.roll_pub = rospy.Publisher("roll", Float64, queue_size=1)
        self.pitch_pub = rospy.Publisher("pitch", Float64, queue_size=1)
        self.yaw_pub = rospy.Publisher("yaw", Float64, queue_size=1)
        self.thrust_pub = rospy.Publisher("thrust", Float64, queue_size=1)
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",Float64,queue_size=1)
        self.lateral_thrust_pub = rospy.Publisher("lateral_thrust",Float64,queue_size=1)
        self.set_state = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        topic = 'vis_mark_array'
        self.publisher = rospy.Publisher(topic, MarkerArray, queue_size=3)
        topic2 = 'vis_mark_array2'
        self.publisher2 = rospy.Publisher(topic2, MarkerArray, queue_size=1)
        topic3 = 'vis_mark_array3'
        self.publisher3 = rospy.Publisher(topic3, MarkerArray, queue_size=1)
        topic4 = 'vis_mark_array4'
        self.publisher4 = rospy.Publisher(topic4, MarkerArray, queue_size=1)
        self.velodyne = rospy.Subscriber('/velodyne_points', PointCloud2, self.velodyne_callback, queue_size=1)
        self.laser = rospy.Subscriber('/bluerov/front_laser/scan', LaserScan, self.laser_callback, queue_size=1)
        #self.odom = rospy.Subscriber('/bluerov/mavros/local_position/odom', Odometry, self.odom_callback, queue_size=1)

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
        #print(self.velodyne_data)

    def laser_callback(self, scan):
        self.last_laser = scan

    def odom_callback(self, od_data):
        self.last_odom = od_data
        #print(self.last_odom)

    # Gestion des thrusters
    def set_thrust(self, value):
        value *= self.thrust_scaler
        self.thrust = max(-1, min(1, value))

    def set_yaw_rate(self, value):
        value *= self.yaw_rate_scaler
        self.yaw_rate = max(-1, min(1, value))

    def set_vertical_thrust(self, value):
        value *= self.vertical_thrust_scaler
        self.vertical_thrust = max(-1, min(1, value))

    def set_lateral_thrust(self, value):
        value *= self.lateral_thrust_scaler
        self.lateral_thrust = max(-1, min(1, value))

    def publish_message(self):
        self.thrust_pub.publish(Float64(self.thrust))
        self.vertical_thrust_pub.publish(Float64(self.vertical_thrust))
        self.lateral_thrust_pub.publish(Float64(self.lateral_thrust))
        self.yaw_pub.publish(Float64(self.yaw_rate))

    # Detect a collision from laser data
    def calculate_observation(self, data):
        min_range = 0.3
        min_laser = 2
        done = False
        col = False

        for i, item in enumerate(data.ranges):
            if min_laser > data.ranges[i]:
                min_laser = data.ranges[i]
            if (min_range > data.ranges[i] > 0):
                done = True
                col = True
        return done, col, min_laser

    # Perform an action and read a new state
    def step(self, act):
        # Publish the robot action
        self.set_thrust(float(act[0]))
        self.set_yaw_rate(float(act[1]))
        rate = rospy.Rate(30.0)
        self.publish_message()
        rate.sleep()

        target = False
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        time.sleep(0.1)

        dataOdom = None
        while dataOdom is None:
            try:
                dataOdom = rospy.wait_for_message('/bluerov/mavros/local_position/odom', Odometry, timeout=0.1)
            except:
                pass

        # Maintient à la bonne profondeur
        if (dataOdom.pose.pose.position.z<-0.1):
            #print("vertical thrust")
            self.set_vertical_thrust(0.1)

        #print("data odom:")
        #print(dataOdom)

        # data = None
        # while data is None:
        #     try:
        #         data = rospy.wait_for_message('/r1/front_laser/scan', LaserScan, timeout=0.1)
        #     except:
        #         pass
        # time.sleep(0.1)
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            pass
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")


        data = self.last_laser

        #dataOdom = self.last_odom
        laser_state = np.array(data.ranges[:])
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        done, col, min_laser = self.calculate_observation(data)



        # Calculate robot heading from odometry data
        self.odomX = dataOdom.pose.pose.position.x
        self.odomY = dataOdom.pose.pose.position.y
        quaternion = Quaternion(
            dataOdom.pose.pose.orientation.w,
            dataOdom.pose.pose.orientation.x,
            dataOdom.pose.pose.orientation.y,
            dataOdom.pose.pose.orientation.z)
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        Dist = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))
        #print("Distance to goal = ", Dist)
        #print("X: ",self.odomX," Y: ",self.odomY)

        # Calculate the angle distance between the robots heading and heading toward the goal
        skewX = self.goalX - self.odomX
        skewY = self.goalY - self.odomY
        dot = skewX * 1 + skewY * 0
        mag1 = math.sqrt(math.pow(skewX, 2) + math.pow(skewY, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skewY < 0:
            if skewX < 0:
                beta = -beta
            else:
                beta = 0 - beta
        beta2 = (beta - angle)
        if beta2 > np.pi:
            beta2 = np.pi - beta2
            beta2 = -np.pi - beta2
        if beta2 < -np.pi:
            beta2 = -np.pi - beta2
            beta2 = np.pi - beta2

        '''Bunch of different ways to generate the reward'''

        # reward = act[0]*0.7-abs(act[1])
        # r1 = 1 - 2 * math.sqrt(abs(beta2 / np.pi))
        # r2 = self.distOld - Dist
        r3 = lambda x: 1 - x if x < 1 else 0.0
        # rl = 0
        # for r in range(len(laser_state[0])):
        #    rl += r3(laser_state[0][r])
        # reward = 0.8 * r1 + 30 * r2 + act[0]/2 - abs(act[1])/2 - r3(min(laser_state[0]))/2
        reward = act[0] / 2 - abs(act[1]) / 2 - r3(min(laser_state[0])) / 2
        # reward = 30 * r2 + act[0] / 2 - abs(act[1]) / 2  # - r3(min(laser_state[0]))/2
        # reward = 0.8 * r1 + 30 * r2

        self.distOld = Dist

        # Detect if the goal has been reached and give a large positive reward
        if Dist < 0.3:
            print("Goal reached!")
            target = True
            done = True
            self.distOld = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))
            reward = 80

        # Detect if ta collision has happened and give a large negative reward
        if col:
            print("Collision!")
            reward = -100

        toGoal = [Dist, beta2, act[0], act[1]]
        state = np.append(laser_state, toGoal)
        return state, reward, done, target

    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        print("Reset ...")
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_proxy()

        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")

        angle = np.random.uniform(-np.pi, np.pi)
        quaternion = Quaternion.from_euler(0., 0., angle)
        object_state = self.set_self_state

        x = 0
        y = 0
        chk = False
        while not chk:
            x = np.random.uniform(-4.5, 4.5)
            y = np.random.uniform(-4.5, 4.5)
            chk = check_pos(x, y)
        object_state.pose.position.x = x
        object_state.pose.position.y = y
        object_state.pose.position.z = -2.5
        object_state.pose.orientation.x = quaternion.x
        object_state.pose.orientation.y = quaternion.y
        object_state.pose.orientation.z = quaternion.z
        object_state.pose.orientation.w = quaternion.w
        self.set_state.publish(object_state)

        self.odomX = object_state.pose.position.x
        self.odomY = object_state.pose.position.y
        #self.odomX = x
        #self.odomY = y
        #print("odomX = ", self.odomX)
        #print("odomY = ", self.odomY)

        self.change_goal()
        self.random_box()
        self.distOld = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))

        data = None
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")
        while data is None:
            try:
                #print("waiting for hukyo...")
                data = rospy.wait_for_message('/bluerov/front_laser/scan', LaserScan, timeout=0.5)
            except:
                pass
        laser_state = np.array(data.ranges[:])
        laser_state[laser_state == inf] = 10
        laser_state = binning(0, laser_state, 20)

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        Dist = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))

        skewX = self.goalX - self.odomX
        skewY = self.goalY - self.odomY

        dot = skewX * 1 + skewY * 0
        mag1 = math.sqrt(math.pow(skewX, 2) + math.pow(skewY, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skewY < 0:
            if skewX < 0:
                beta = -beta
            else:
                beta = 0 - beta
        beta2 = (beta - angle)

        if beta2 > np.pi:
            beta2 = np.pi - beta2
            beta2 = -np.pi - beta2
        if beta2 < -np.pi:
            beta2 = -np.pi - beta2
            beta2 = np.pi - beta2

        toGoal = [Dist, beta2, 0.0, 0.0]
        state = np.append(laser_state, toGoal)
        return state

    # Place a new goal and check if its lov\cation is not on one of the obstacles
    def change_goal(self):
        if self.upper < 10:
            self.upper += 0.004
        if self.lower > -10:
            self.lower -= 0.004

        gOK = False

        while not gOK:
            self.goalX = self.odomX + random.uniform(self.upper, self.lower)
            self.goalY = self.odomY + random.uniform(self.upper, self.lower)
            gOK = check_pos(self.goalX, self.goalY)

        if gOK:
            box_state = ModelState()
            box_state.model_name = 'goal_box'
            box_state.pose.position.x = self.goalX
            box_state.pose.position.y = self.goalY
            box_state.pose.position.z = -2.4
            box_state.pose.orientation.x = 0.0
            box_state.pose.orientation.y = 0.0
            box_state.pose.orientation.z = 0.0
            box_state.pose.orientation.w = 1.0
            self.set_state.publish(box_state)


    # Randomly change the location of the boxes in the environment on each reset to randomize the training environment
    def random_box(self):
        for i in range(4):
            name = 'cardboard_box_' + str(i)

            x = 0
            y = 0
            chk = False
            while not chk:
                x = np.random.uniform(-6, 6)
                y = np.random.uniform(-6, 6)
                chk = check_pos(x, y)
                d1 = math.sqrt((x - self.odomX) ** 2 + (y - self.odomY) ** 2)
                d2 = math.sqrt((x - self.goalX) ** 2 + (y - self.goalY) ** 2)
                if d1 < 1.5 or d2 < 1.5:
                    chk = False
            box_state = ModelState()
            box_state.model_name = name
            box_state.pose.position.x = x
            box_state.pose.position.y = y
            box_state.pose.position.z = -2.6
            box_state.pose.orientation.x = 0.0
            box_state.pose.orientation.y = 0.0
            box_state.pose.orientation.z = 0.0
            box_state.pose.orientation.w = 1.0
            self.set_state.publish(box_state)
