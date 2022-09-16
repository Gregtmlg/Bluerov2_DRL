#!/usr/bin/env python
import rospy
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
from bluerov_sim.msg import RangeMeasurementArray, RangeMeasurement


class RangeSensorNode():
    def __init__(self):
        rospy.init_node("range_sensor")

        self.range_pub = rospy.Publisher("ranges",
                                         RangeMeasurementArray,
                                         queue_size=1)
        self.simulation = rospy.get_param("~sim", True)

        if self.simulation:
            rospy.Subscriber("tag_detections_sim",
                             RangeMeasurementArray,
                             self.on_sim_dist,
                             queue_size=1)
        else:
            rospy.Subscriber("tag_detections",
                             AprilTagDetectionArray,
                             self.on_apriltag,
                             queue_size=1)

    def on_sim_dist(self, msg):
        self.range_pub.publish(msg)

    def on_apriltag(self, tag_array_msg):
        num_tags = len(tag_array_msg.detections)

        range_array_msg = RangeMeasurementArray()
        range_array_msg.header = tag_array_msg.header

        # if tags are detected
        if num_tags:
            for i, tag in enumerate(tag_array_msg.detections):
                range_msg = RangeMeasurement()

                # tag id -> our numbers for each anchor (we lost tag 1...)
                if tag.id[0] == 0:
                    range_msg.id = 1
                elif tag.id[0] == 2:
                    range_msg.id = 2
                elif tag.id[0] == 3:
                    range_msg.id = 3
                elif tag.id[0] == 4:
                    range_msg.id = 4

                range_msg.range = np.sqrt(tag.pose.pose.pose.position.x**2 +
                                          tag.pose.pose.pose.position.y**2 +
                                          tag.pose.pose.pose.position.z**2)
                range_msg.header = tag_array_msg.header
                range_array_msg.measurements.append(range_msg)

        self.range_pub.publish(range_array_msg)


def main():
    node = RangeSensorNode()

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == "__main__":
    main()
