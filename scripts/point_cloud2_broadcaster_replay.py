#!/usr/bin/env python

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import Range, PointField, PointCloud2
from tf2_geometry_msgs import PointStamped
import sys
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import Pose
from numpy import ceil


class Cloud:
    def __init__(self, num_points, min_data_points=360):
        rospy.logerr('Cloud num of points: %s', str(num_points))
        self.num_points = num_points
        self.ps = PointStamped()
        self.ps_transformed = PointStamped()
        self.cloud = PointCloud2()
        self.stacking_factor = int(ceil(min_data_points / num_points))
        rospy.logwarn('Number of stacking to botain dense cloud: %s', str(self.stacking_factor))
        self.stacking_iter = 0
        self.num_points = num_points
        self.points = []
        self.seq = 0
        self.xm = 0.
        self.ym = 0.
        self.cloud_pub = rospy.Publisher('cloud_in', PointCloud2, queue_size=3)

    def callback(self, msg, bot_id):
        if bot_id < 10:
            frame_id = 'bot0' + str(bot_id) + "_analyt"
        else:
            frame_id = 'bot' + str(bot_id) + "_analyt"
        self.ps.header.frame_id = frame_id
        self.ps.header.stamp = rospy.Time(0)
        self.ps.point.x = msg.range
        self.ps.point.y = 0
        self.ps.point.z = 0

        try:
            self.ps_transformed = tfBuffer.transform(self.ps, "odom", timeout=rospy.Duration(1))
            r = int(self.ps_transformed.point.x * 255.0)
            g = int(self.ps_transformed.point.y * 255.0)
            b = int(self.ps_transformed.point.z * 255.0)
            a = (self.ps_transformed.point.x ** 2 + self.ps_transformed.point.y ** 2 + self.ps_transformed.point.z ** 2) ** 0.5
            pt = [self.ps_transformed.point.x, self.ps_transformed.point.y, self.ps_transformed.point.z, r, g, b, a]
            self.points.append(pt)
            if len(self.points) == (self.stacking_factor * self.num_points):
                self.cloud = self.point_cloud(np.array(self.points), "odom")
                self.points = []
                self.cloud.header.frame_id = "odom"
                self.cloud.header.seq = self.seq
                self.cloud.header.stamp = rospy.Time.now()
                cloud_transformed = transform_cloud(cloud_in=self.cloud, target_frame="bot_center",
                                                    time=rospy.Time(0))
                self.cloud_pub.publish(cloud_transformed)
                self.seq += 1

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # raise
            rospy.loginfo('waiting for transform msgs.')
            pass

    def set_mean_pose(self, msg):
        self.xm = msg.position.x
        self.ym = msg.position.y

    def point_cloud(self, points, parent_frame):
        """ Creates a point cloud message.
        Args:
            points: Nx7 array of xyz positions (m) and rgba colors (0..1)
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        """
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize

        data = points.astype(dtype).tobytes()

        fields = [PointField(
            name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyzrgba')]
        header = PointCloud2().header
        header.frame_id = parent_frame
        return PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 7),
            row_step=(itemsize * 7 * points.shape[0]),
            data=data
        )


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


def transform_cloud_full(cloud_in, target_frame, source_frame, time, timeout=1):
    try:
        trans = tfBuffer.lookup_transform_full(
            target_frame=target_frame,
            target_time=time,
            source_frame=source_frame,
            source_time=time,
            fixed_frame='world',
            timeout=rospy.Duration(timeout)
        )
    except tf2_ros.LookupException as ex:
        rospy.logwarn(ex)
        return
    except tf2_ros.ExtrapolationException as ex:
        rospy.logwarn(ex)
        return
    cloud_out = do_transform_cloud(cloud_in, trans)

    return cloud_out


if __name__ == '__main__':
    rospy.init_node('point_cloud_broadcaster')
    num_points = int(sys.argv[1])
    try:
        min_data_points = int(sys.argv[2])
        cloud = Cloud(num_points, min_data_points)
    except:
        cloud = Cloud(num_points)
    tfBuffer = tf2_ros.Buffer(rospy.Duration(5))
    tf2_ros.TransformListener(tfBuffer)
    rospy.Subscriber("bot00_pose_offset", Pose, cloud.set_mean_pose)
    for i in range(num_points):
        if i < 10:
            key = str(0) + str(i)
        else:
            key = str(i)
        rospy.Subscriber("bot%s/dist/data" % key, Range, cloud.callback, i, queue_size=num_points + 1)
    rospy.spin()
