#!/usr/bin/env python
import rospy
import rosnode
from mavros_msgs.srv import CommandBool


class AutoArmNode():
    """This node arms the vehicle in the simulation environment. It should not
    be started for the realworld vehicle. An armed vehicle is allowed to actuate
    its thrusters.
    """
    def __init__(self):
        rospy.init_node("auto_arm")
        self.arm_vehicle()

    def arm_vehicle(self):
        rospy.wait_for_service("mavros/cmd/arming")
        rospy.sleep(0.0000001)
        nodes = rosnode.get_node_names()
        if "/gazebo" not in nodes:
            rospy.loginfo("Not running in simulation. Auto arming disabled.")
            return
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        while not arm(True).success:
            rospy.logwarn_throttle(1, "Could not arm vehicle. Keep trying.")
        rospy.loginfo("Armed successfully.")

    def disarm_vehicle(self):
        rospy.wait_for_service("mavros/cmd/arming")
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        arm(False)


def main():
    AutoArmNode()


if __name__ == "__main__":
    main()
