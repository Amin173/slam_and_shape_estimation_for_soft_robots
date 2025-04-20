
#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu
import numpy as np
from std_msgs.msg import String
import sys
from time import sleep
from scipy.signal import butter, lfilter, lfiltic


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, a, b, z=None):
    if isinstance(z, type(None)):
            z = lfiltic(b, a, data)
            y = data
    else:
        y, z = lfilter(b, a, np.array(data).reshape(-1), zi=z)
    return y, z

class odomBroadcaster:

    pose_covariance = [0.005, 0.0, 0.0, 0.0, 0.0, 0.0,
                                               0.0, 0.005, 0.0, 0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0000, 0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0, 0.0000, 0.0, 0.0,
                                               0.0, 0.0, 0.0, 0.0, 0.0000, 0.0,
                                               0.0, 0.0, 0.0, 0.0, 0.0, 0.00001]

    twist_covariance = [0.00001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.00001, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0000, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0000, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0000, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]

    offsets_exp_imu = 1 / np.array([ 0.7568297 +0.64634099j, 0.96707903+0.20240087j, -0.40572821-0.90640207j,
                                    0.11281897-0.98843956j, 0.79984378-0.59549041j, 0.5362549 -0.83934994j,
                                    0.61015607+0.79047816j, 0.67759265-0.7333606j, -0.0401674 +0.99429693j,
                                    0.82817443+0.54636922j, -0.09855799-0.98938747j, -0.07407743+0.99360817j]) * np.exp(-1j*15/180*np.pi)

    counter = 0

    def __init__(self, num_nodes=12, estimator_model='linear', data_source='simulation'):

        self.num_of_bots = num_nodes
        self.store_data = []

        # define initial pose variables, positions and velocities
        self.xy = np.array([1.5, .5])#np.array([float(rospy.get_param('initial_pose/x')), float(rospy.get_param('initial_pose/y'))])
        self.th = 0.#float(rospy.get_param('initial_pose/a'))
        self.vxvy = np.array([0., 0.])
        self.vth = 0.0

        # Subscriber data variables: angles are stored as complex variables
        self.th_imu_data = np.ones(self.num_of_bots) + 0j
        self.vth_imu_data = np.zeros(self.num_of_bots)
        self.th_imu_times = np.array([rospy.Time.now().to_sec()] * self.num_of_bots)

        self.xy_april_data = np.zeros((self.num_of_bots, 2))
        self.th_april_data = np.ones(self.num_of_bots) + 0j
        self.vxvy_april_data = np.zeros((self.num_of_bots, 2))
        self.vth_april_data = np.zeros(self.num_of_bots)

        self.control_dir = np.array([0, -1])
        self.control_actions = np.array(self.num_of_bots)

        # Filters
        fs = 5         #Copy from simulated_robot_tcp.py
        cutoff = .25    #Hz
        self.filter_b, self.filter_a = butter_lowpass(cutoff, fs, order=2)
        self.imu_zi = [None] * self.num_of_bots

        # Timing variables for subscribers
        tmp = rospy.Time.now()
        self.time_imu_list = [tmp] * self.num_of_bots
        self.time_april = tmp
        self.time_odom = tmp

        self.r = rospy.Rate(20)

        self.startup_sequence = True

        # TF variables:
        
        self.odom_broadcaster = tf.TransformBroadcaster()
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        # self.odom_broadcaster.sendTransform(
        #     (self.xy[0], self.xy[1], 0.0),
        #     odom_quat,
        #     rospy.Time.now(),
        #     "bot_center",
        #     "odom"
        # )
        self.t = TransformStamped()
        self.t.header.frame_id = self.key(self.num_of_bots)
        
        # Variables to optimize model:
        # Max distance between bots
        self.lmax = .15/2 * np.tan(np.pi / self.num_of_bots) / np.sin(np.pi / 12)
        self.heading_angle_offsets = np.exp(-1j * np.pi / 2 * (np.arange(self.num_of_bots) % 2))

        if type(estimator_model) == None:
            self.estimator_model = 'linear'
        else:
            self.estimator_model = estimator_model

        if type(data_source) == None:
            self.data_source = 'simulation'
        else:
            self.data_source = data_source

        # Publishers
        self.odom_publisher = rospy.Publisher("odom/vel_model", Odometry, queue_size=5)

        #self.pub_ground_truth = rospy.Publisher("odom/vel_model", PoseWithCovarianceStamped, queue_size=5)
        self.pub_ground_truth = rospy.Publisher("pose/ground_truth", PoseWithCovarianceStamped, queue_size=5)
        #self.pub_ground_truth = rospy.Publisher("pose/ground_truth", Odometry, queue_size=5)

        # Subscribers
        # Subscriber for apriltags
        rospy.Subscriber("state", String, self.update_april_data)
        if self.data_source == 'experimental':
            rospy.Subscriber("control_cmd", String, self.update_control_command)

        # Subscrber for imu
        for i in range(self.num_of_bots):
            idx = self.key(i)
            rospy.Subscriber("bot%s/imu/data" % idx, Imu, self.update_imu_data, idx, queue_size=self.num_of_bots * 2)

        # Subscriber for controller
        rospy.Subscriber("locomotion_stats", String, self.update_des_dir)

        # TF listener
        self.tf_listener = tf.TransformListener()

        
        

    # update imu data
    def update_imu_data(self, msg, idx):
        # Bot number
        i = int(idx)
        
        tmp = rospy.Time.now()
        dt = (tmp - self.time_imu_list[i]).to_sec()
        self.time_imu_list[i] = tmp

        # Convert quaternion to angle
        ang_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        # Store result for the specified bot
        if self.data_source == 'experimental':
            euler = np.exp(1j * tf.transformations.euler_from_quaternion(ang_list)[-1]) / self.offsets_exp_imu[i]
            self.th_imu_times[i] = tmp.to_sec()
        else:
            euler = np.exp(-1j * tf.transformations.euler_from_quaternion(ang_list)[-1])
            
        # Apply filter:
        vel = np.angle(euler / self.th_imu_data[i]) / dt
        vel, self.imu_zi[i] =  butter_lowpass_filter(vel, self.filter_a, self.filter_b, self.imu_zi[i])

        # Store results
        self.vth_imu_data[i] = vel
        self.th_imu_data[i] = euler 

    # Update apriltag data
    def update_april_data(self, msg):
        # First calculate increment in time:
        tmp = rospy.Time.now()
        dt = (tmp - self.time_april).to_sec()
        self.time_april = tmp

        # Convert data to usable type
        data_dict = eval(msg.data)
        tmp = data_dict[self.key(self.num_of_bots)]
        xy0 = np.array(tmp[:2])
        th0 = np.array(tmp[2])
        xy_l = np.empty((2, 0), float)
        th_l = np.empty((0), float)
        for i in range(self.num_of_bots):
            data_tmp = data_dict[self.key(i)]
            xyi = np.array(data_tmp[:2]) - xy0
            if self.data_source == 'experimental':
                xyi /= 100
                xyi[1] = -xyi[1]
                thi = np.exp(-1j * (np.array(data_tmp[2]) - th0) * np.pi / 180)
            else:
                thi = np.exp(-1j * (np.array(data_tmp[2]) - th0) * np.pi / 180)

            xy_l = np.hstack((xy_l, xyi.reshape((2, -1))))
            th_l = np.hstack((th_l, [thi]))

        # store data into lists and calculate velocity
        self.vxvy_april_data = (xy_l.T - self.xy_april_data) / dt
        self.xy_april_data = xy_l.T

        self.vth_april_data = np.angle(th_l / self.th_april_data) / dt
        self.th_april_data = th_l

        # use apriltag data as initial position for the first step
        if self.startup_sequence:
            self.startup_sequence = False
            self.xy = np.average(self.xy_april_data, axis=0)
            self.th_imu_data = self.th_april_data

            self.publish_rel_poisitions(self.xy_april_data, self.th_april_data * self.heading_angle_offsets)
            self.publish_ground_truth()
            

    # Update controller info
    def update_des_dir(self, msg):
        data_dict = eval(msg.data)
        if self.data_source == 'simulated':
            self.control_dir = np.array(data_dict['dir'])
            self.control_actions = np.array(data_dict['actions'])
        else:
            self.control_dir = np.array([data_dict['dir'][0], -data_dict['dir'][1]])

    def update_control_command(self, msg):
        if self.data_source == 'experimental':
            data_dict = eval(msg.data)

            tmp = []
            for i in range(self.num_of_bots):
                tmp.append( -np.array(data_dict[self.key(i)][0]))
            
            self.control_actions = np.array(tmp)


    # Estimates the relative positions of bots given the heading directions
    def analyt_model(self, X, best_fit=True):
        # Bot normal direction angles
        X = X.reshape(self.num_of_bots) * np.exp(-1j * np.pi / 2)
        
        ### Precalculate variables that will later be nidded
        # Calculate difference in angle, constrain between - pi to pi
        diff = np.angle(np.roll(X, -1) / X)

        # Calculate direction to next bot:
        theta =  - (np.roll(X, -1) + X)
        theta /= np.abs(theta)

        # Method to calculate distance between bots
        if self.estimator_model == 'rigid_joint':
            L = 2 * self.lmax * np.cos(.5 * diff)
        elif self.estimator_model == 'linear':         # ransac regressor
            L = (155.93 - 0.8547 * 180 / np.pi * np.abs(diff)) / 1000. * 12 / self.num_of_bots
        elif self.estimator_model == 'linear2':         # ransac regressor
            L = (158.842 - 0.937 * 180 / np.pi * np.abs(diff)) / 1000. * 12 / self.num_of_bots
        else:
            L = 12**2 / self.num_of_bots/1000

        Ct = np.real(theta)#np.cos(theta)
        St = np.imag(theta)#np.sin(theta)
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


    def key(self, i):
        if i < 10:
            key = str(0) + str(i)
        else:
            key = str(i)
        return key

    # Publishes the position of all sub units relative to the odometry link, Slam will then change the position of the odom
    def publish_rel_poisitions(self, positions, angles):
        # generate name
        for i in range(self.num_of_bots):
            idx = self.key(i)

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, np.angle(angles[i]))
            self.odom_broadcaster.sendTransform(
                (positions[i, 0], positions[i, 1], 0.0),
                odom_quat,
                rospy.Time.now(),
                "bot" + idx + "_analyt",
                "bot_center"
            )

    def publish_ground_truth(self):
        # generate name
        angles = self.th_april_data * self.heading_angle_offsets
        positions = self.xy_april_data
        for i in range(self.num_of_bots):
            idx = self.key(i)

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, np.angle(angles[i]))
            self.odom_broadcaster.sendTransform(
                (positions[i, 0], positions[i, 1], 0.0),
                odom_quat,
                rospy.Time.now(),
                "bot" + idx + "_true",
                "map"
            )

        
        # odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        # self.odom_broadcaster.sendTransform(
        #     (np.average(positions[:, 0]), np.average(positions[:, 1]), 0.0),
        #     odom_quat,
        #     rospy.Time.now(),
        #     "bot_center",
        #     "map"
        # )

        ground_truth = PoseWithCovarianceStamped()
        ground_truth.header.frame_id = 'map'
        ground_truth.header.stamp = rospy.Time.now()
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        ground_truth.pose.pose.position.x = np.average(positions[:, 0])
        ground_truth.pose.pose.position.y = np.average(positions[:, 1])
        ground_truth.pose.pose.position.z = 0
        ground_truth.pose.pose.orientation.x = q[0]
        ground_truth.pose.pose.orientation.y = q[1]
        ground_truth.pose.pose.orientation.z = q[2]
        ground_truth.pose.pose.orientation.w = q[3]
        pose_covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0000, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0000, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0000, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]

        ground_truth.pose.covariance = pose_covariance
        self.pub_ground_truth.publish(ground_truth)

    def estimate_odometry(self, positions, angles):
        # Get time from prevous odometry
        current_time = rospy.Time.now()
        dt = (current_time - self.time_odom).to_sec()
        self.time_odom = current_time

        # Calculate desired direction velocity
        des_dir = self.control_dir * .015 / 2
        # Convert angles to wheel directions, then calculate the forces produced based on the control actions
        n = np.vstack((np.imag(angles / self.heading_angle_offsets),
                       np.real(angles / self.heading_angle_offsets))).T
        forces = n.T * self.control_actions * 0.0034#0.000424 #0.011

        # Convert actions into estimated results
        est_translation = des_dir #np.average(forces, axis=1)

        true_velocity = np.linalg.norm(np.average(self.vxvy_april_data, axis=0))
        #rospy.logerr("Average scalar: %s", str(true_velocity / np.linalg.norm(est_translation)))

        r = positions - np.average(positions, axis=0)
        est_rotation = np.diagonal(r @ np.array([[0, 1], [-1, 0]]) @ forces)[np.arange(self.num_of_bots) % 2 == 0]
        est_rotation = 0*np.average(est_rotation)

        # Calculate odometry position and rotation
        self.xy += est_translation * dt
        self.th += est_rotation * dt
        

        # Calculate odometry velocity and rotational velocity
        self.vxvy = est_translation
        self.vth = est_rotation


        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_broadcaster.sendTransform(
                (self.xy[0], self.xy[1], 0.0),
                odom_quat,
                current_time,
                "bot_center",
                "odom"
            )     

        self.odom_broadcaster.sendTransform(
                (self.xy[0], self.xy[1], 0.0),
                odom_quat,
                current_time,
                "odom2",
                "map"
            )
            

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.xy[0], self.xy[1], 0.), Quaternion(*odom_quat))
        odom.pose.covariance = self.pose_covariance

        # set the velocity
        odom.child_frame_id = "bot_center"
        odom.twist.twist = Twist(
           Vector3(est_translation[0], est_translation[1], 0),
           Vector3(0, 0, est_rotation))
        odom.twist.covariance = self.twist_covariance

        # publish the message
        self.odom_publisher.publish(odom)

    # Calculates next step position of base link
    def update(self):

        # First wait for initial data to appear:
        while self.startup_sequence:
          sleep(.01)

        while not rospy.is_shutdown():
            
            # Get angles for robot
            if self.data_source == "experimental":
                self.th_imu_data *= np.exp(1j * self.vth_imu_data * ((rospy.Time.now()).to_sec() - self.th_imu_times))
                angles = self.th_imu_data * self.heading_angle_offsets
            else:
                #self.th_imu_data *= np.exp(1j * self.vth_imu_data * ((rospy.Time.now()).to_sec() - self.th_imu_times))
                angles = self.th_imu_data * self.heading_angle_offsets

            # Calculate positions of subunits and set center as middle position
            p_rel = self.analyt_model(angles, True)
            p_rel -= np.average(p_rel, axis=0)

            # Publish relative positions
            self.publish_rel_poisitions(p_rel, angles)

            #self.publish_ground_truth()

            # Now calculate odometry:
            self.estimate_odometry(p_rel, angles)


            try:
                
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/bot_center', rospy.Time(0))
                trans = np.array(trans)[:2]
                rot = tf.transformations.euler_from_quaternion(rot)[-1]
                
                c = np.cos(rot)
                s = np.sin(rot)
                R = np.array([[c, s], [-s, c]])

                p_l = (R @ p_rel.T).T + trans
                t_l = angles * np.exp(1j * rot)
                t_l = np.vstack((np.real(angles), np.imag(angles))).T
                ttrue_l = np.vstack((np.real(self.th_april_data * self.heading_angle_offsets),
                                            np.imag(self.th_april_data * self.heading_angle_offsets))).T

                save_data = np.concatenate((self.xy_april_data.flatten(), self.vxvy_april_data.flatten(),
                                            ttrue_l.flatten(),
                                            np.array(self.control_actions).flatten(), np.array(self.control_dir).flatten(),
                                            p_l.flatten(), t_l.flatten()))
                self.store_data.append(save_data)
            except:
                rospy.logerr('Could not store simulation data')

            self.r.sleep()
        #self.store_data = np.array(self.store_data)
        
        np.savetxt('/opt/ros/overlay_ws/RAL-files/Test.csv', self.store_data[3:], delimiter=',')


if __name__ == '__main__':
    sleep(1.5)
    rospy.init_node('odometry_publisher')
    try:
        odom_pub = odomBroadcaster(int(sys.argv[1]), sys.argv[2], sys.argv[3])
        #odom_pub = odomBroadcaster(24, "rigid_joint", "simulated")
        #odom_pub = odomBroadcaster(12, "linear", "experimental")
    except:
        odom_pub = odomBroadcaster()
    odom_pub.update()
