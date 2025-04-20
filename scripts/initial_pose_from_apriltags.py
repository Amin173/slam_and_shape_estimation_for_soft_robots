#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from ruamel.yaml import YAML
from pathlib import Path
import tf2_ros
import sys

class InitialPose:
    def __init__(self, num_tags):
        self.num_tags = num_tags
        if num_tags > 9:
            self.num_tags_str = str(num_tags)
        else:
            self.num_tags_str = '0' + str(num_tags)

        self.num_bots = num_tags - 1
        if self.num_bots > 9:
            self.num_bots_str = str(self.num_bots)
        else:
            self.num_bots_str = '0' + str(self.num_bots)

        self.isSet = False
        self.x0, self.y0, self.th0 = [0, 0, 0]
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.yaml = YAML()
        self.path = Path('/opt/ros/overlay_ws/src/slam_and_shape_estimation_for_soft_robots/config/config.yml')

    def update(self, data):
        data_dict = eval(data.data)
        self.x0 = (data_dict['00'][0] - data_dict[self.num_bots_str][0]) / 100
        self.y0 = -(data_dict['00'][1] - data_dict[self.num_bots_str][1]) / 100
        self.th0 = -(data_dict['00'][2] - data_dict[self.num_bots_str][2]) * 3.14 / 180
        if not (self.x0 == 0 or self.y0 == 0 or self.th0 == 0):
            rospy.set_param('/config/initial_pose/x', self.x0)
            rospy.set_param('/config/initial_pose/y', self.y0)
            rospy.set_param('/config/initial_pose/a', self.th0)
            self.isSet = True


if __name__ == '__main__':
    try:
        num_tags = int(sys.argv[1])
    except:
        num_tags = 12
    rospy.init_node('dynamic_tf2_broadcaster')
    initial_pose = InitialPose(num_tags)
    rospy.Subscriber("state", String, initial_pose.update)
    rate = rospy.Rate(0.1)
    while not initial_pose.isSet:
        rospy.logwarn(f"Waiting for rosbag data...")
        rate.sleep()
    initial_pose.yaml.dump(rospy.get_param("/config"), initial_pose.path)
    rospy.logwarn(f"Initial pose set: {initial_pose.isSet}")
    rospy.logwarn("Configuration complete!")

