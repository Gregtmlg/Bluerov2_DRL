import gym
from gym import spaces

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
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_srvs.srv import Empty
import os
import time


os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'



# Function to put the laser data in bins
def binning(lower_bound, data, quantity):
    width = round(len(data) / quantity)
    quantity -= 1
    bins = []
    for low in range(lower_bound, lower_bound + quantity * width + 1, width):
        bins.append(min(data[low:low + width]))
    return np.array([bins])

class UnityEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self, launchfile, max_episode_length=256):

        self.step_counter = 0

        self.goalX = 1
        self.goalY = 0.0

        self.upper = 5.0
        self.lower = -5.0
        self.last_odom = None
        self.last_pose = None
        self.last_vel = None

        self._max_episode_length = max_episode_length

        # Define action and observation space
        gym.Env.__init__(self)
        self.action_space = spaces.Box(low=-1, high=1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=float("inf"), high=float("inf"), shape=(16,), dtype=np.float32)

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

        self.goal_state = PoseStamped()

        self.set_self_state = ModelState()
        self.set_self_state.model_name = 'uuv_bluerov2_heavy_0'
        self.set_self_state.pose.position.x = 0.
        self.set_self_state.pose.position.y = 0.
        self.set_self_state.pose.position.z = 0.
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 0.0

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
        self.goal_pose_publish= rospy.Publisher("goal_pose", PoseStamped, queue_size=1)
        self.odom = rospy.Subscriber('/bluerov/mavros/local_position/odom', Odometry, self.odom_callback, queue_size=1)  
        self.bluerov_pose = rospy.Subscriber('/bluerov/mavros/local_position/pose', PoseStamped, self.bluerov_pose_callback, queue_size=1)
        self.bluerov_velocity = rospy.Subscriber('/bluerov/mavros/local_position/velocity_local', TwistStamped, self.bluerov_vel_callback, queue_size=1)


        self.change_goal()



    def bluerov_pose_callback(self, pose_data):
        self.last_pose = pose_data

    def bluerov_vel_callback(self, vel_data):
        self.last_vel = vel_data

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
    def get_observation(self):
        pose = None
        while pose is None:
            try:
                pose = rospy.wait_for_message('/bluerov/mavros/local_position/pose', PoseStamped, timeout=0.1)
            except:
                rospy.loginfo('ça passe pas get obs')
        velocity = rospy.wait_for_message('/bluerov/mavros/local_position/velocity_local', TwistStamped, timeout=0.2)
        bluerov_world_position = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        bluerov_world_orientation = np.array([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        bluerov_linear_velocity = np.array([velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z])
        bluerov_angular_velocity = np.array([velocity.twist.angular.x, velocity.twist.angular.y, velocity.twist.angular.z])
        goal_world_position = np.array([self.goal_state.pose.position.x, self.goal_state.pose.position.y, self.goal_state.pose.position.z])

        return np.concatenate([bluerov_world_position,
                               bluerov_world_orientation,
                               bluerov_linear_velocity,
                               bluerov_angular_velocity,
                               goal_world_position,])

    



    def step(self, action):
        # Execute one time step within the environment
        rospy.loginfo('début de step')
        self.step_counter += 1
        rospy.loginfo("juste avant le wait_fr_msg")
        previous_bluerov_position = None
        while previous_bluerov_position is None:
            try:
                previous_bluerov_position = rospy.wait_for_message('/bluerov/mavros/local_position/pose', PoseStamped, timeout=0.1)
            except: 
                rospy.loginfo('ça passe pas step')
        rospy.loginfo("juste après le wait_fr_msg")

        # Publish the robot action
        self.set_thrust(float(action[0]))
        self.set_yaw_rate(float(action[1]))
        rate = rospy.Rate(30.0)
        self.publish_message()
        rate.sleep()

        target = False

        dataOdom = None
        while dataOdom is None:
            try:
                dataOdom = rospy.wait_for_message('/bluerov/mavros/local_position/pose', PoseStamped, timeout=0.1)
            except:
                pass

        # Maintient à la bonne profondeur
        if (dataOdom.pose.position.z<-0.1):
            #print("vertical thrust")
            self.set_vertical_thrust(0.1)


        observations = self.get_observation()
        info = {}
        target = False

        #on teste si on a dépassé le nombre de step max d'un épisode
        if self.step_counter >= self._max_episode_length:
            target = True

        current_bluerov_position = None
        while current_bluerov_position is None:
            try:
                current_bluerov_position = rospy.wait_for_message('/bluerov/mavros/local_position/pose', PoseStamped, timeout=0.1)
            except:
                pass

        goal_world_position = np.array([self.goal_state.pose.position.x, self.goal_state.pose.position.y, self.goal_state.pose.position.z])
        current_bluerov_position_array = np.array([current_bluerov_position.pose.position.x, current_bluerov_position.pose.position.y, current_bluerov_position.pose.position.z])
        previous_bluerov_position_array = np.array([previous_bluerov_position.pose.position.x, previous_bluerov_position.pose.position.y, previous_bluerov_position.pose.position.z])

        previous_dist_to_goal = np.linalg.norm(goal_world_position- previous_bluerov_position_array)
        current_dist_to_goal = np.linalg.norm(goal_world_position- current_bluerov_position_array)

        reward = previous_dist_to_goal - current_dist_to_goal
        if current_dist_to_goal < 0.3:
            target = True

        rospy.loginfo(self.step_counter)
        return observations, reward, target, info




    def reset(self):
        # Reset the state of the environment to an initial state
        print("Reset ...")
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_proxy()

        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")

        self.step_counter = 0

        angle = np.random.uniform(-np.pi, np.pi)
        quaternion = Quaternion.from_euler(0., 0., angle)
        object_state = self.set_self_state

        x = 0.0
        y = 0.0
        z = 0.0
        object_state.pose.position.x = x
        object_state.pose.position.y = y
        object_state.pose.position.z = z
        object_state.pose.orientation.x = quaternion.x
        object_state.pose.orientation.y = quaternion.y
        object_state.pose.orientation.z = quaternion.z
        object_state.pose.orientation.w = quaternion.w
        self.set_state.publish(object_state)


        self.change_goal()

        observations = self.get_observation()


        return observations

    # Place a new goal and check if its lov\cation is not on one of the obstacles
    def change_goal(self):
        alpha = 2 * math.pi * np.random.rand()
        r = 1.00 * math.sqrt(np.random.rand()) + 0.20
        self.goalX = math.sin(alpha) * r
        self.goalY = math.cos(alpha) * r

        self.goal_state.pose.position.x = self.goalX
        self.goal_state.pose.position.y = self.goalY
        self.goal_state.pose.position.z = 0.05
        self.goal_state.pose.orientation.x = 0.0
        self.goal_state.pose.orientation.y = 0.0
        self.goal_state.pose.orientation.z = 0.0
        self.goal_state.pose.orientation.w = 1.0
        self.goal_pose_publish.publish(self.goal_state)




    