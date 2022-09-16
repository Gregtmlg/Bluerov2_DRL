#!/usr/bin/env python

from enum import auto
from typing_extensions import Self
import rospy
from mavros_msgs.msg import MotorSetpoint
import random
import time

import pygame
import rospkg
import os
from std_msgs.msg import Float64

class CmdAuto():

    def __init__(self) -> None:
        self.rate = [0.0, 0.25, 0.5, 0.75, 1.0]
        pygame.init()
        pygame.mixer.quit()
        self.thrust = 0.0
        self.thrust_stepsize = 0.1
        self.thrust_scaler = 0.4
        self.lateral_thrust = 0.0
        self.lateral_thrust_stepsize = 0.1
        self.lateral_thrust_scaler = 0.4
        self.vertical_thrust = 0.0
        self.vertical_thrust_stepsize = 0.1
        self.vertical_thrust_scaler = 0.4
        self.yaw_rate = 0.0
        self.yaw_rate_stepsize = 0.1
        self.yaw_rate_scaler = 0.2



        self.roll_pub = rospy.Publisher("roll", Float64, queue_size=1)
        self.pitch_pub = rospy.Publisher("pitch", Float64, queue_size=1)
        self.yaw_pub = rospy.Publisher("yaw", Float64, queue_size=1)
        self.thrust_pub = rospy.Publisher("thrust", Float64, queue_size=1)
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                   Float64,
                                                   queue_size=1)
        self.lateral_thrust_pub = rospy.Publisher("lateral_thrust",
                                                  Float64,
                                                  queue_size=1)

    def set_rate(self):
        return random.choice(self.rate)

    def run(self):
        rate = rospy.Rate(30.0)
        self.publish_message()
        rate.sleep()



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

    def increase_thrust_scaler(self, value):
        self.thrust_scaler += value
        self.thrust_scaler = max(0, min(1, self.thrust_scaler))

    def increase_yaw_rate_scaler(self, value):
        self.yaw_rate_scaler += value
        self.yaw_rate_scaler = max(0, min(1, self.yaw_rate_scaler))

    def increase_vertical_thrust_scaler(self, value):
        self.vertical_thrust_scaler += value
        self.vertical_thrust_scaler = max(0, min(1,
                                                 self.vertical_thrust_scaler))

    def increase_lateral_thrust_scaler(self, value):
        self.lateral_thrust_scaler += value
        self.lateral_thrust_scaler = max(0, min(1, self.lateral_thrust_scaler))

    # actions
    def turn_left(self):
        self.set_yaw_rate(self.set_rate())
        self.set_vertical_thrust(0)
        self.set_lateral_thrust(0)
        self.set_thrust(0)
    
    def turn_right(self):
        self.set_yaw_rate(-self.set_rate())
        self.set_vertical_thrust(0)
        self.set_lateral_thrust(0)
        self.set_thrust(0)

    def go_up(self):
        self.set_vertical_thrust(self.set_rate())
        self.set_lateral_thrust(0)
        self.set_thrust(0)
        self.set_yaw_rate(0)
    
    def go_down(self):
        self.set_vertical_thrust(-self.set_rate())
        self.set_lateral_thrust(0)
        self.set_thrust(0)
        self.set_yaw_rate(0)

    def go_left(self):
        self.set_lateral_thrust(self.set_rate())
        self.set_vertical_thrust(0)
        self.set_thrust(0)
        self.set_yaw_rate(0)

    
    def go_right(self):
        self.set_lateral_thrust(-self.set_rate())
        self.set_vertical_thrust(0)
        self.set_thrust(0)
        self.set_yaw_rate(0)

    
    def forward(self):
        self.set_thrust(self.set_rate())
        self.set_vertical_thrust(0)
        self.set_lateral_thrust(0)
        self.set_yaw_rate(0)


    def backward(self):
        self.set_thrust(-self.set_rate())
        self.set_vertical_thrust(0)
        self.set_lateral_thrust(0)
        self.set_yaw_rate(0)

    # def init_controls(self):
    #     controls = {
    #         "turn_left":
    #         dict(
    #             description="Turn left.",
    #             pressed_callback=(lambda: self.set_yaw_rate(1)),
    #             released_callback=(lambda: self.set_yaw_rate(0)),
    #         ),
    #         "turn_right":
    #         dict(
    #             description="Turn right.",
    #             pressed_callback=(lambda: self.set_yaw_rate(-1)),
    #             released_callback=(lambda: self.set_yaw_rate(0)),
    #         ),
    #         "go_up":
    #         dict(
    #             description="Positive vertical thrust.",
    #             pressed_callback=(lambda: self.set_vertical_thrust(1)),
    #             released_callback=(lambda: self.set_vertical_thrust(0)),
    #         ),
    #         "go_down":
    #         dict(
    #             description="Negative vertical thrust.",
    #             pressed_callback=(lambda: self.set_vertical_thrust(-1)),
    #             released_callback=(lambda: self.set_vertical_thrust(0)),
    #         ),
    #         "go_left":
    #         dict(
    #             description="Positive lateral thrust.",
    #             pressed_callback=(lambda: self.set_lateral_thrust(1)),
    #             released_callback=(lambda: self.set_lateral_thrust(0)),
    #         ),
    #         "go_right":
    #         dict(
    #             description="Negative lateral thrust.",
    #             pressed_callback=(lambda: self.set_lateral_thrust(-1)),
    #             released_callback=(lambda: self.set_lateral_thrust(0)),
    #         ),
    #         "foreward":
    #         dict(
    #             description="Forward thrust.",
    #             pressed_callback=(lambda: self.set_thrust(1)),
    #             released_callback=(lambda: self.set_thrust(0)),
    #         ),
    #         "backward":
    #         dict(
    #             description="Backward thrust.",
    #             pressed_callback=(lambda: self.set_thrust(-1)),
    #             released_callback=(lambda: self.set_thrust(0)),
    #         ),
    #     }
    #     return controls

    def get_resource_path(self):
        res_path = rospkg.RosPack().get_path("bluerov_sim")
        res_path = os.path.join(res_path, "res")
        return res_path

    def print_current_values(self):
        print("thrust: {}\nyaw_rate: {}\nlateral_thrust: {}\n"
              "vertical_thrust: {}".format(self.thrust, self.yaw_rate,
                                           self.lateral_thrust,
                                           self.vertical_thrust))


    def publish_message(self):
        self.thrust_pub.publish(Float64(self.thrust))
        self.vertical_thrust_pub.publish(Float64(self.vertical_thrust))
        self.lateral_thrust_pub.publish(Float64(self.lateral_thrust))
        self.yaw_pub.publish(Float64(self.yaw_rate))


if __name__ == "__main__":
    rospy.init_node("test_cmd_vel")
    # rospy.loginfo("J'attends")
    # rospy.wait_for_message("mavros/setpoint_motor/setpoint", MotorSetpoint, timeout=10)
    # rospy.loginfo("C'EST BON !!")
    while not rospy.is_shutdown():
        autocmd = CmdAuto()
        autocmd.turn_left()
        autocmd.run()
        time.sleep(2)
        autocmd.forward()
        autocmd.run()
        time.sleep(2)

