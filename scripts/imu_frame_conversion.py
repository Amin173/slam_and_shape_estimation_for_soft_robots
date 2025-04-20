#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Imu

rospy.init_node('imu_frame_conversion', anonymous=True)
num_of_bots_str = sys.argv[1]
num_of_bots = int(num_of_bots_str)

def key(i):
    if i < 10:
        key = str(0) + str(i)
    else:
        key = str(i)
    return key

def publish_imu(msg):
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = str(msg.header.frame_id) + "_analyt"
    imu_msg.header.seq = msg.header.seq
    id_num = int(str(msg.header.frame_id)[3:])
    imu_msg.orientation.x = msg.orientation.x
    imu_msg.orientation.y = msg.orientation.y
    imu_msg.orientation.z = msg.orientation.z
    imu_msg.orientation.w = msg.orientation.w
    if id_num % 2 == 0:
        imu_msg.linear_acceleration.x = -msg.linear_acceleration.y/100
        imu_msg.linear_acceleration.y = msg.linear_acceleration.x/100
        imu_msg.linear_acceleration.z = 0
    else:
        imu_msg.linear_acceleration.x = -msg.linear_acceleration.x/100
        imu_msg.linear_acceleration.y = -msg.linear_acceleration.y/100
        imu_msg.linear_acceleration.z = 0
    imu_pub[key(id_num)].publish(imu_msg)

imu_pub = {}
for i in range(num_of_bots):
    imu_pub[key(i)] = rospy.Publisher("bot%s/imu_conv/data" % key(i), Imu, queue_size=1)
    rospy.Subscriber("bot%s/imu/data" % key(i), Imu, publish_imu)

rate = rospy.Rate(20)
while not rospy.is_shutdown():

    rate.sleep()
