#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry


class VisualizerNode():
    def __init__(self):
        rospy.init_node("rviz_visualizer")
        self.bluerov_marker_pub = rospy.Publisher("bluerov_marker",
                                                  Marker,
                                                  queue_size=30)
        self.ground_truth_sub = rospy.Subscriber("ground_truth/state", Odometry,
                                                 self.on_ground_truth)

    def run(self):
        rospy.spin()

    def publish_marker(self, pose, header):
        msg = Marker()
        msg.header = header
        msg.ns = "bluerov_ns"
        msg.id = 27
        msg.type = Marker.MESH_RESOURCE
        msg.pose = pose
        msg.mesh_resource = "package://bluerov_sim/models/uuv_bluerov2_heavy/meshes/BlueROV2heavy.dae"
        msg.color.a = 1.0
        msg.color.r = 1.0
        msg.color.g = 1.0
        msg.color.b = 1.0
        msg.scale.x = 1
        msg.scale.y = 1
        msg.scale.z = 1
        self.bluerov_marker_pub.publish(msg)

    def on_ground_truth(self, msg):
        self.publish_marker(msg.pose.pose, msg.header)


def main():
    node = VisualizerNode()
    node.run()


if __name__ == "__main__":
    main()
