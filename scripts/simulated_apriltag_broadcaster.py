#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from numpy import array, cos, sin, pi, sqrt, arctan2, loadtxt
import numpy as np
from scipy.spatial.transform import Rotation as R

import datetime


def _rotation_matrix_to_euler_angles(Rot):

    sy = sqrt(Rot[0, 0] * Rot[0, 0] + Rot[1, 0] * Rot[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = arctan2(Rot[2, 1], Rot[2, 2])
        y = arctan2(-Rot[2, 0], sy)
        z = arctan2(Rot[1, 0], Rot[0, 0])
    else:
        x = arctan2(-Rot[1, 2], Rot[1, 1])
        y = arctan2(-Rot[2, 0], sy)
        z = 0

    return z * 180 / pi#array([x, y, z])

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


def main(num_of_tags, csv_filename):
    # Say if you want to lot the data after the test
    plot_data = False
    animate_data = True
    date = datetime.datetime.now()

    # Load csv file:
    num_bots = int(num_of_tags) - 1
    tmp_data = loadtxt('/opt/ros/overlay_ws/src/slam_and_shape_estimation_for_soft_robots/simulated_csv/' + csv_filename, delimiter=',')
    max_time = tmp_data[-1, 0]
    

    # Create data dictionary and fake data for now
    data = {}

    # Generate offset point
    
    p0 = array([0, 0, 0.])

    # Define dummy data for case where csv is not used
    ang_2_q = lambda alf: R.from_euler('z', -alf, degrees=False).as_matrix()
    
    # Create all node data
    for i in range(int(num_of_tags)):
        # Generate tag
        idx = str(i)
        if len(idx) == 1:
            idx = '0' + idx
        
        # Generate position
        alf = 2 * pi * i / int(num_of_tags)
        p = 30. * array([cos(alf), sin(alf), 0.]) + p0
        
        # Generate angle:
        ang = _rotation_matrix_to_euler_angles(ang_2_q((2 * pi * i / (int(num_of_tags) - 1)) - pi / 2 * (i%2)) )
        p[-1] = ang

        if i == num_bots:
            p = p0

        data[idx] = tuple(p)

    data['time'] = -1

    # Setup ROS # state is a tuple of x, y, and angle
    pub = rospy.Publisher('state', String, queue_size=10)
    rospy.init_node('AprilTags', anonymous=False)


    rate = rospy.Rate(10) # in Hz

    tmp = rospy.get_rostime()
    t0 = tmp.secs + tmp.nsecs * 1e-9
    while not rospy.is_shutdown():# and rospy.get_rostime().secs < max_time:
        now = rospy.get_rostime()
        now = now.secs + now.nsecs * 1e-9 - t0
        positions, angles, ranges, range_types = getTime_data(now, num_bots, tmp_data)
        #angles = np.flip(angles)
        angles = - angles

        data['time'] = t0#now

        # For each node update data
        for i, k in enumerate(list(data.keys())[:-2]):
            angle = (angles[i] - pi/2 * (i%2)) * 180 / pi
            if angle > 180:
                angle -= 360
            elif angle < -180:
                angle += 360
            data[k] = (positions[i, 0] - p0[0], positions[i, 1] - p0[1], angle)

        # Publish result
        pub.publish(str(data))
        rate.sleep()


if __name__ == '__main__':
    try:
        main(sys.argv[1], sys.argv[2])
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass
