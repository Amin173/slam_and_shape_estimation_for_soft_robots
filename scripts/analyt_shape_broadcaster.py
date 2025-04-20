#!/usr/bin/env python

import rospy
import tf_conversions
from tf.transformations import *
import sys
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose
import tf
from std_msgs.msg import String
import numpy as np

class AnalyticModel:
    def __init__(self, frame_id, num_of_bots, origin_tag_id, estimator_model='linear'):
        self.num_of_bots = int(num_of_bots)
        self.listener = tf.TransformListener()
        self.origin_tag = origin_tag_id
        x0 = float(rospy.get_param('initial_pose/x'))
        y0 = float(rospy.get_param('initial_pose/y'))
        th0 = float(rospy.get_param('initial_pose/a'))
        self.base_link_pose_x = x0
        self.base_link_pose_y = y0
        self.base_link_orientation = th0
        self.rel_rotation_set = [False] * self.num_of_bots
        self.cam_trnsfrm_recieved = False
        self.heading_angles = np.array([])
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.t.header.frame_id = frame_id
        self.imu_quaternions = np.zeros((self.num_of_bots, 4))
        self.imu_quaternion_offsets = np.zeros((self.num_of_bots, 4))
        self.heading_angles_rel = np.zeros(self.num_of_bots)
        self.heading_angle_offsets = 90 * (np.arange(self.num_of_bots) % 2)
        self.heading_angle_offsets_exp = np.exp(1j * self.heading_angle_offsets)
        self.x_rel = np.zeros(self.num_of_bots)
        self.y_rel = np.zeros(self.num_of_bots)
        self.pose_offset_pub = rospy.Publisher("bot00_pose_offset", Pose, queue_size=50)
        self.data = {}
        
        # Variables to optimize model:
        # Max distance between bots
        self.lmax = .15/2 * np.tan(np.pi / self.num_of_bots) / np.sin(np.pi / 12) * 100 #150/20#20 

        if type(estimator_model) == None:
            self.estimator_model = 'linear'
        else:
            self.estimator_model = estimator_model

    def transform_broadcaster(self, bot_id):
        self.t.child_frame_id = "bot" + bot_id + "_analyt"
        bot_number = int(bot_id)
        self.t.header.stamp = rospy.Time.now()
        self.t.transform.translation.x = self.x_rel[bot_number] + self.base_link_pose_x
        self.t.transform.translation.y = self.y_rel[bot_number] + self.base_link_pose_y
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, (
                -self.heading_angle_offsets[bot_number] - self.heading_angles_rel[
            bot_number]) * np.pi / 180)
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]
        # self.t.transform.rotation.x = self.imu_quaternions[bot_id, 0]
        # self.t.transform.rotation.y = self.imu_quaternions[bot_id, 1]
        # self.t.transform.rotation.z = self.imu_quaternions[bot_id, 2]
        # self.t.transform.rotation.w = self.imu_quaternions[bot_id, 3]
        self.br.sendTransform(self.t)



    def base_link_robot_loc(self, msg):
        self.base_link_pose_x = msg.transform.rotation.x
        self.base_link_pose_y = msg.transform.rotation.y
        e = tf_conversions.transformations.euler_from_quaternion_from(msg.transform.rotation.x,
                                                                      msg.transform.rotation.y,
                                                                      msg.transform.rotation.z,
                                                                      msg.transform.rotation.w)
        self.base_link_orientation = e[2]



    def update_imu_quaternions(self, msg):
        bot_id = msg.header.frame_id
        bot_number = int(bot_id[3:])
        if (not self.rel_rotation_set[bot_number]) and self.cam_trnsfrm_recieved:
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, -self.heading_angles_rel[
                bot_number] * np.pi / 180 + self.base_link_orientation)
            q1_inv = [msg.orientation.x, msg.orientation.y, msg.orientation.z, - msg.orientation.w]
            q2 = [q[0], q[1], q[2], q[3]]
            self.imu_quaternion_offsets[bot_number, :] = tf.transformations.quaternion_multiply(q2, q1_inv)
            self.rel_rotation_set[bot_number] = True
        self.imu_quaternions[bot_id, :] = tf.transformations.quaternion_multiply(self.qr,
                                                                                 [msg.orientation.x, msg.orientation.y,
                                                                                  msg.orientation.z, msg.orientation.w])
        bot_id = msg.header.frame_id
        bot_number = int(bot_id[3:])
        if (not self.rel_rotation_set[bot_number]) and self.cam_trnsfrm_recieved:
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, -self.heading_angles_rel[
                bot_number] * np.pi / 180 + self.base_link_orientation)
            q1_inv = [msg.orientation.x, msg.orientation.y, msg.orientation.z, - msg.orientation.w]
            q2 = [q[0], q[1], q[2], q[3]]
            self.imu_quaternion_offsets[bot_number, :] = tf.transformations.quaternion_multiply(q2, q1_inv)
            self.rel_rotation_set[bot_number] = True
        self.imu_quaternions[bot_id, :] = tf.transformations.quaternion_multiply(self.qr,
                                                                                 [msg.orientation.x, msg.orientation.y,
                                                                                  msg.orientation.z, msg.orientation.w])

    def analyt_model(self, X, best_fit=True):
        # Vector for distances between subunits
        L = []
        
        # Vector with direction angle between one bot and the next
        theta = []

        # Bot normal direction angles
        X = X.reshape(self.num_of_bots)
        
        
        ### Precalculate variables that will later be nidded
        # Calculate angles, offset 90 deg for odd numbered bots and constrain between - pi to pi
        normals = (X + self.heading_angle_offsets) * np.pi / 180
        normals += (normals > np.pi) * (- 2 * np.pi) + (normals < - np.pi) * (2 * np.pi)

        # Calculate difference in angle, constrain between - pi to pi
        diff = np.roll(normals, -1) - normals
        diff += (diff > np.pi) * (- 2 * np.pi) + (diff < - np.pi) * (2 * np.pi)

        # Calculate direction to next bot:
        theta = np.exp(1j * np.roll(normals, -1)) + np.exp(1j * normals)
        theta = np.angle(theta) - np.pi / 2
        #theta += (theta > np.pi) * (- 2 * np.pi) + (theta < - np.pi) * (2 * np.pi)
        

        # Method to calculate distance between bots
        if self.estimator_model == 'rigid_joint':
            L = 2 * self.lmax * np.cos(.5 * diff)
        elif self.estimator_model == 'linear':         # ransac regressor
            L = (155.93 - 0.8547 * 180 / np.pi * np.abs(diff)) / 10.
        else:
            L = self.num_of_bots

        Ct = np.cos(theta)#np.cos(theta)
        St = np.sin(theta)#np.sin(theta)
        # Distance vector from one bot to the next
        B = np.vstack((Ct, St))
        
        # Apply best fit to the estimator model
        if best_fit:
            C = Ct.reshape((-1, self.num_of_bots))
            S = St.reshape((-1, self.num_of_bots))

            # Calculate the inverse of the covariances
            COVi = np.eye(self.num_of_bots)

            # Values to be used in operation
            CCT = np.sum(C**2)#(C @ COVi @ C.T)[0, 0]
            SST = np.sum(S**2)#(S @ COVi @ S.T)[0, 0]
            SCT = np.sum(C*S)#(S @ COVi @ C.T)[0, 0]
            CST = SCT#(C @ COVi @ S.T)[0, 0]

            P = (SCT / CCT * C - S) / (SCT * CST / CCT - SST)
            Q = (C - CST * P) / CCT

            # Apply adjustment
            #x = - COVi @ (C.T @ Q @ L + S.T @ P @ L)
            x = - ((C.T @ Q + S.T @ P) @ L)
            L += x
            
        
        # Update vectors that take you from one bot to the next
        B = (L * B).T

        # calculate the positions of each bot
        rel_positions = np.vstack((np.zeros((1, 2)), np.cumsum(B, axis=0)[:-1, :]))
        return rel_positions

    def update_aprilt_state_values(self, data):
        data_dict = eval(data.data)
        # self.base_link_pose_x = (data_dict['00'][0] - data_dict[self.origin_tag][0]) / 100
        # self.base_link_pose_y = -(data_dict['00'][1] - data_dict[self.origin_tag][1]) / 100
        # self.base_link_orientation = -(data_dict['00'][2] - data_dict[self.origin_tag][2]) * 3.14 / 180
        self.data = data_dict
        for i in range(self.num_of_bots):
            # self.heading_angles_rel[i] = (data_dict[self.key(i)][2] - data_dict['00'][2])
            self.heading_angles_rel[i] = (data_dict[self.key(i)][2])
        analyt_input = np.array(self.heading_angles_rel).reshape((1, self.num_of_bots))
        rel_poses = self.analyt_model(analyt_input)
        self.x_rel = rel_poses[:, 0] / 100
        self.y_rel = - rel_poses[:, 1] / 100
        for i in range(1, self.num_of_bots):
            self.transform_broadcaster(self.key(i))
        pose_offset = Pose()
        pose_offset.position.x = np.mean(self.x_rel) - self.x_rel[0]
        pose_offset.position.y = np.mean(self.y_rel) - self.y_rel[0]
        pose_offset.position.z = -(data_dict['00'][2] - data_dict[self.origin_tag][2])
        self.pose_offset_pub.publish(pose_offset)

    def key(self, i):
        if i < 10:
            key = str(0) + str(i)
        else:
            key = str(i)
        return key


def bot_id(i):
    if i < 10:
        id1 = "bot" + str(0) + str(i)
    else:
        id1 = "bot" + str(i)
    return id1


if __name__ == '__main__':
    rospy.init_node('dynamic_tf2_broadcaster', anonymous=True)
    listener = tf.TransformListener()
    num_of_bots = sys.argv[1]
    frame_id = sys.argv[2]
    origin_tag = sys.argv[3]
    estimator_model = sys.argv[4]

    # num_of_bots = 24#sys.argv[1]
    # frame_id = "odom"#sys.argv[2]
    # origin_tag = "24"#sys.argv[3]
    # estimator_model = "rigid_joint"#sys.argv[4]

    # child_frames = rospy.get_param('~child_frames')
    broadcaster = AnalyticModel(frame_id, num_of_bots, origin_tag, estimator_model)
    # for i in child_frames:
    #     rospy.Subscriber('%s/imu/data' % bot_id(i), Imu, broadcaster.update_imu_quaternions)
    rospy.Subscriber("state", String, broadcaster.update_aprilt_state_values)
    # rospy.Subscriber("odometry/filtered_map",
    #                  Odometry,
    #                  broadcaster.base_link_odom)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #rospy.logerr("Analyt_model, Number of bots: %s", str(broadcaster.num_of_bots))
        try:
            (trans, rot) = listener.lookupTransform('/odom', '/bot00_analyt', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            broadcaster.base_link_pose_x = trans[0]
            broadcaster.base_link_pose_y = trans[1]
            broadcaster.base_link_orientation = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
