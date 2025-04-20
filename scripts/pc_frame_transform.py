#!/usr/bin/env python

import rospy
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from slam_and_shape_estimation_for_soft_robots.srv import PointCloud2Request

def transform_cloud(cloud_in, target_frame, time, timeout=1):
    try:
        trans = tfBuffer.lookup_transform(target_frame, cloud_in.header.frame_id,
                                          time,
                                          rospy.Duration(timeout))
    except tf2_ros.LookupException as ex:
        rospy.logwarn(ex)
        return
    except tf2_ros.ExtrapolationException as ex:
        rospy.logwarn(ex)
        return
    cloud_out = do_transform_cloud(cloud_in, trans)

    return cloud_out


def handle_point_cloud_transform(req):
    cloud_transformed = transform_cloud(cloud_in=req.cloud, target_frame=req.target_frame,
                                        time=rospy.Time(0))

    return cloud_transformed


def pc_frame_transform_server():

    s = rospy.Service('point_cloud_transform', PointCloud2Request, handle_point_cloud_transform)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node('pc_frame_transform_server')
    tfBuffer = tf2_ros.Buffer(rospy.Duration(5))
    tf2_ros.TransformListener(tfBuffer)
    pc_frame_transform_server()
