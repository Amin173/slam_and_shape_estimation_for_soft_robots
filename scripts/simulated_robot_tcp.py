#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Range, Imu
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R
from numpy import pi, loadtxt
from numpy.random import random
import ast
import numpy as np

# Get initial arguments, this should include csv file location to run from simulated data
# number of tags, number of robots + 1
num_tags = int(sys.argv[1])
num_bots = int(sys.argv[2])
csv_filename = sys.argv[3]

# Load data

csv_data = loadtxt('/opt/ros/overlay_ws/src/slam_and_shape_estimation_for_soft_robots/simulated_csv/' + csv_filename, delimiter=',')
max_time = csv_data[-1, 0]

def num_2_str(num):
    if num > 9:
        return str(num)
    else:
        return '0' + str(num)

# Define dummy data for case where csv is not used
ang_2_q = lambda alf: R.from_euler('z', -alf, degrees=False).as_quat()
default_dict = lambda n: {"acc": [0., 0., -1.], "rot": ang_2_q((2 * pi * n / (num_tags - 1)) - pi / 2 * (n%2)), "dist": 300 + (random()*100 - 50), 'type': 1}

# Returns the positions, angles and ranges at a given time for all bots
def getTime_data(time, num_bots, data):
    # get data timestep
    file_dt = data[1, 0] - data[0, 0]

    # calculate index for given time and timestep:
    idx = int(np.round(time / file_dt * 1., 0))

    tmp = data[idx, 1:].reshape((num_bots, 8))

    positions = tmp[:, :2]
    velocities = tmp[:, 2:4]
    norms = tmp[:, 4:6]
    angles = np.arctan2(norms[:, 1], norms[:, 0])
    ranges = tmp[:, 6]
    range_type = tmp[:, 7]

    # return needed data only
    return positions, angles, ranges, range_type

# Initialize node
rospy.init_node('tcp_connection', anonymous=True)

# Define publishers: num_tags * 3
range_pubs = []
imu_pubs = []
data_pubs = []
for i in range(num_tags):

    range_pub = rospy.Publisher("bot%s/dist/data" % num_2_str(i), Range, queue_size=1)
    imu_pub = rospy.Publisher("bot%s/imu/data" % num_2_str(i), Imu, queue_size=1)
    data_pub = rospy.Publisher("bot%s/sensor/data" % num_2_str(i), String, queue_size=1)

    range_pubs.append(range_pub)
    imu_pubs.append(imu_pub)
    data_pubs.append(data_pub)

loc_publisher = rospy.Publisher("locomotion_stats", String, queue_size=10)


# Function to publish locomotion from desired direction data:
def publish_locomotion(direction, states):
    global loc_publisher

    locomotion_stats = {"mode": 'move', "dir": direction, "actions": states}
    loc_publisher.publish(str(locomotion_stats))

# Function to publish imu data
def publish_imu(publisher, dev_id_str, imu_measurement, seq):
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = 'bot' + dev_id_str
    imu_msg.header.seq = seq
    imu_msg.linear_acceleration.x = imu_measurement["acc"][0]
    imu_msg.linear_acceleration.y = -imu_measurement["acc"][1]
    imu_msg.linear_acceleration.z = imu_measurement["acc"][2]
    imu_msg.orientation.x = imu_measurement["rot"][0]
    imu_msg.orientation.y = imu_measurement["rot"][1]
    imu_msg.orientation.z = imu_measurement["rot"][2]
    imu_msg.orientation.w = imu_measurement["rot"][3]
    publisher.publish(imu_msg)

# Function to publish range data
def publish_range(publisher, dev_id_str, range_measurement, seq):
    dist_msg = Range()
    h = rospy.Header()
    h.stamp = rospy.Time.now()
    h.frame_id = 'bot' + dev_id_str
    h.seq = seq
    dist_msg.header = h
    dist_msg.radiation_type = 1 # int(range_measurement['type'])
    dist_msg.field_of_view = 0.2
    dist_msg.min_range = 0
    dist_msg.max_range = 4
    dist_msg.range = float(range_measurement['dist']) / 1000

    if int(range_measurement['type']) == 1:
        publisher.publish(dist_msg)


# Code starts here:
# time to which send csv data
glob_time = -1

def callbackTime(msg):
    global glob_time
    tmp = ast.literal_eval(msg.data)
    glob_time = tmp["time"]

# Callback to send range data
rospy.Subscriber("state", String, callbackTime)

# Define control algorithm:
# Possible target direction
quads = np.array([np.cos(np.linspace(0, np.pi * 2 - np.pi / 6, 12)),
                    np.sin(np.linspace(0, np.pi * 2 - np.pi / 6, 12))]).T
current_quad = 9
current_spin = 1
distance_threshold = 0.05

def calculateDistancesInDirections(normals, ranges, range_types):
    global quads
    global current_quad

    cosines = np.dot(normals[range_types == 1, :], quads.T)
    proj_distances = np.multiply(ranges[range_types == 1], cosines.T)
    #cosines = np.dot(normals, quads.T)
    #proj_distances = np.multiply(ranges, cosines.T)
    average_distances = []
    for c, d in zip(cosines.T, proj_distances):
        av_dist = np.average(d[c > np.cos(np.pi / 4)])
        average_distances.append(av_dist)

    return [average_distances[(current_quad - 2) % 12],
            average_distances[(current_quad - 1) % 12],
            average_distances[current_quad],
            average_distances[(current_quad + 1) % 12],
            average_distances[(current_quad + 2) % 12]]

def getNextControl(angles, ranges, range_types):
    global quads
    global current_quad
    global current_spin
    global distance_threshold
    global norm_bots

    norm_bots = np.vstack((np.cos(angles), np.sin(angles))).T

    av_distances = calculateDistancesInDirections(norm_bots, ranges, range_types)
    if av_distances[2] < distance_threshold:
        diff = np.argmax(av_distances) - 2
        current_quad += int(np.sign(diff))
        current_quad = current_quad % 12

        if diff * current_spin > 0:
            current_spin *= -1

    ref = quads[current_quad, :]

    # get desired velocities
    tmp = np.dot(norm_bots, ref)
    actions = np.sign(tmp) * 1
    actions[ranges < distance_threshold / 1000.] = -1
    actions[np.arange(num_bots) % 2 == 1] = - current_spin

    return actions




# Define rate at which to run simulation
actions = np.zeros(num_bots)
seq = 0
rate = rospy.Rate(20)
now = -1.
while not rospy.is_shutdown() and now < max_time:
    # increment sequence time
    seq += 1

    if glob_time >= 0.:

        now = rospy.get_rostime()
        now = now.secs + now.nsecs * 1e-9 - glob_time
        
        positions, angles, ranges, range_types = getTime_data(now, num_bots, csv_data)
        #ranges = np.flip(ranges)
        
        if seq % 2 == 0:
            actions = getNextControl(angles, ranges, range_types)

        if seq > 10 and seq % 4 == 0:
            publish_locomotion(quads[current_quad], actions)

            # for each tag generate data and publish
            for i, (imu_p, range_p, imu_d, range_d, type_d) in enumerate(zip(imu_pubs, range_pubs, angles, ranges, range_types)):
                std2 = (4.71 / 180 * np.pi)**2
                imu_d += np.random.vonmises(0, 1 / std2)
                data_dict = {"acc": [0., 0., -1.], "rot": ang_2_q((imu_d + pi / 2 * (i%2))), "dist": range_d * 1000, "type": type_d}
                
                dev_id_str = num_2_str(i)

                # Publish data for tag
                publish_imu(imu_p, dev_id_str, data_dict, seq)
                publish_range(range_p, dev_id_str, data_dict, seq)
    rate.sleep()