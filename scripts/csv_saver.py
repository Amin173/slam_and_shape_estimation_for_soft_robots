#!/usr/bin/env python

import rospy
from tf.transformations import *
import tf
import pandas as pd
from os.path import exists
from geometry_msgs.msg import PoseWithCovarianceStamped


class dataSaver:

    def __init__(self):
        self.df = pd.DataFrame()
        self.saving_path = "/opt/ros/overlay_ws/RAL-files/slam_localization_t0.csv"
        counter = 0
        while exists(self.saving_path):
            counter += 1
            path = self.saving_path.split("_t")
            self.saving_path = path[0] + "_t" + str(counter) + ".csv"

        self.df = pd.DataFrame(columns=["time", "x_est", "y_est", "rot_est", "x_gt", "y_gt", "rot_gt"])
        self.x_est = None
        self.y_est = None
        self.rot_est = None
        self.x_gt = None
        self.y_gt = None
        self.rot_gt = None
        self.time = rospy.Time.now().to_sec()

    def callback(self, msg):
        try:
            (trans_estimate, rot_estimate) = listener.lookupTransform('/map', '/bot_center', rospy.Time(0))
            euler_est = tf.transformations.euler_from_quaternion(rot_estimate)
            data.x_est = trans_estimate[0]
            data.y_est = trans_estimate[1]
            data.rot_est = euler_est[2]
            data.time = rospy.Time.now().to_sec()
            euler_gt = tf.transformations.euler_from_quaternion(
                [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
            data.x_gt = msg.pose.pose.position.x
            data.y_gt = msg.pose.pose.position.y
            data.rot_gt = euler_gt[2]

            data.store()


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def store(self):
        self.df = self.df.append(
            {"time": self.time, "x_est": self.x_est, "y_est": self.y_est, "rot_est": self.rot_est, "x_gt": self.x_gt,
             "y_gt": self.y_gt, "rot_gt": self.rot_gt}, ignore_index=True)

    def save(self):
        self.df.to_csv(self.saving_path)


if __name__ == '__main__':
    rospy.init_node('csv_saver', anonymous=True)
    data = dataSaver()
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    rospy.Subscriber("pose/ground_truth", PoseWithCovarianceStamped, data.callback)
    while not rospy.is_shutdown():
        rate.sleep()

    data.save()
