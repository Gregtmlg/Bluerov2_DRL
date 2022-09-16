from __future__ import print_function

import random
import rospy

from geometry_msgs.msg import PoseStamped

from unity_robotics_demo_msgs.srv import PositionService, PositionServiceResponse



def goal_pose_callback(pose):
    goalX = pose.pose.position.x
    goalY = pose.pose.position.y
    return [goalX, goalY]

def new_position(req):
    print("Request: \n{}".format(req.input))
    goal_subsciber = rospy.Subscriber('goal_pose', PoseStamped, goal_pose_callback, queue_size=1)
    goal = None
    while goal is None:
        try:
            goal = rospy.wait_for_message('goal_pose', PoseStamped, timeout=0.1)
        except:
            rospy.loginfo("Goal waiting for order")
    req.input.pos_x = goal[0]
    req.input.pos_z = goal[1]

    return PositionServiceResponse(req.input)


def translate_position_server():
    rospy.init_node('position_server')
    while not rospy.is_shutdown():
        s = rospy.Service('pos_srv', PositionService, new_position)
        print("Ready to move cubes!")


if __name__ == "__main__":
    translate_position_server()