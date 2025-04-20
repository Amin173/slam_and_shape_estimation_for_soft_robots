#!/usr/bin/env python

from sensor_msgs.msg import  PointField, PointCloud2
from geometry_msgs.msg import Pose
import sensor_msgs.point_cloud2 as pc2
import rospy
import open3d as o3d
import numpy as np
import struct
import ctypes
import cv2
from math import sin, cos, radians, atan2, degrees, hypot
import math
from slam_and_shape_estimation_for_soft_robots.srv import PointCloud2Request


class HoghmanFilter:

    def __init__(self):
        self.cloud_xyz = None
        self.x_min, self.x_max, self.y_min, self.y_max = [None] * 4
        self.W = None
        self.L = None
        # Create black empty images
        self.size = [None] * 3
        self.window = "Drawing 1"
        self.image = None
        self.lines = None
        self.houghman_minLineLength = 50
        self.houghman_maxLineGap = 100
        self.houghman_threshold = 2
        self.corner_threshold = 50
        self.merge_lines = False
        self.min_distance_to_merge = 100
        self.min_angle_to_merge = 20
        self.parallel_threshold = 10

    def set_cloud(self, cloud_xyz):
        self.cloud_xyz = cloud_xyz
        self.x_min, self.x_max, self.y_min, self.y_max = self.cloud_xyz[:, 0].min(), self.cloud_xyz[:,
                                                                                     0].max(), self.cloud_xyz[:,
                                                                                               1].min(), self.cloud_xyz[
                                                                                                         :,
                                                                                                         1].max()
        l = (self.y_max - self.y_min) / (self.x_max - self.x_min)
        self.W = 800
        self.L = int(self.W / l)
        # Create black empty images
        self.size = self.W, self.L, 3
        self.image = np.zeros(self.size, dtype=np.uint8)
        for index, row in enumerate(self.cloud_xyz):
            self.my_filled_circle(self.image, self.point2pixel(row[0], row[1]))
        self.lines = self.apply_filter()

    def my_filled_circle(self, img, center):
        thickness = -1
        line_type = 8
        cv2.circle(img,
                   center,
                   self.W // 512,
                   (255, 255, 255),
                   thickness,
                   line_type)

    def point2pixel(self, x, y):
        px = int((x - self.x_min) * self.L / (self.x_max - self.x_min))
        py = int((y - self.y_min) * self.W / (self.y_max - self.y_min))
        return (px, py)

    def pixel2point(self, px, py):
        x = px * (self.x_max - self.x_min) / self.L + self.x_min
        y = py * (self.y_max - self.y_min) / self.W + self.y_min
        return (x, y)

    def apply_filter(self):
        dst = cv2.Canny(self.image, 0, 255)
        # dst = cv2.cvtColor(atom_image, cv2.COLOR_BGR2GRAY)
        lines = cv2.HoughLinesP(dst, rho=1, theta=np.pi / 180, threshold=self.houghman_threshold,
                                minLineLength=self.houghman_minLineLength,
                                maxLineGap=self.houghman_maxLineGap)
        if self.merge_lines:
            lines = self.merge(lines)
        return lines

    def merge(self, lines):
        _lines = []
        lines = np.reshape(lines, [-1, 1, 4]).tolist()
        for _line in self.get_lines(lines):
            _lines.append([(_line[0], _line[1]), (_line[2], _line[3])])
        # sort
        _lines_x = []
        _lines_y = []
        for line_i in _lines:
            orientation_i = atan2((line_i[0][1] - line_i[1][1]), (line_i[0][0] - line_i[1][0]))
            if (abs(degrees(orientation_i)) > 45) and abs(degrees(orientation_i)) < (90 + 45):
                _lines_x.append(line_i)
            else:
                _lines_x.append(line_i)

        _lines_x = sorted(_lines_x, key=lambda _line: _line[0][0])
        # _lines_y = sorted(_lines_y, key=lambda _line: _line[0][1])
        merged_lines_x = self.merge_lines_pipeline_2(_lines_x)
        merged_lines_y = self.merge_lines_pipeline_2(_lines_y)

        merged_lines_all = []
        merged_lines_all.extend(merged_lines_x)
        merged_lines_all.extend(merged_lines_y)
        merged_lines_all = np.reshape(merged_lines_all, [-1, 4])
        return merged_lines_all

    def get_lines(self, lines_in):
        if cv2.getVersionString() < '3.0':
            return lines_in[0]
        return [l[0] for l in lines_in]

    def lines_close(self, line1, line2):
        dist1 = hypot(line1[0][0] - line2[0][0], line1[0][0] - line2[0][1])
        dist2 = hypot(line1[0][2] - line2[0][0], line1[0][3] - line2[0][1])
        dist3 = hypot(line1[0][0] - line2[0][2], line1[0][0] - line2[0][3])
        dist4 = hypot(line1[0][2] - line2[0][2], line1[0][3] - line2[0][3])

        if (min(dist1, dist2, dist3, dist4) < 100):
            return True
        else:
            return False

    def lineMagnitude(self, x1, y1, x2, y2):
        lineMagnitude = math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))
        return lineMagnitude

    # Calc minimum distance from a point and a line segment (i.e. consecutive vertices in a polyline).
    def distancePointLine(self, px, py, x1, y1, x2, y2):
        LineMag = self.lineMagnitude(x1, y1, x2, y2)

        if LineMag < 0.00000001:
            DistancePointLine = 9999
            return DistancePointLine

        u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)))
        u = u1 / (LineMag * LineMag)

        if (u < 0.00001) or (u > 1):
            # // closest point does not fall within the line segment, take the shorter distance
            # // to an endpoint
            ix = self.lineMagnitude(px, py, x1, y1)
            iy = self.lineMagnitude(px, py, x2, y2)
            if ix > iy:
                DistancePointLine = iy
            else:
                DistancePointLine = ix
        else:
            # Intersecting point is on the line, use the formula
            ix = x1 + u * (x2 - x1)
            iy = y1 + u * (y2 - y1)
            DistancePointLine = self.lineMagnitude(px, py, ix, iy)

        return DistancePointLine

    def get_distance(self, line1, line2):
        dist1 = self.distancePointLine(line1[0][0], line1[0][1],
                                       line2[0][0], line2[0][1], line2[1][0], line2[1][1])
        dist2 = self.distancePointLine(line1[1][0], line1[1][1],
                                       line2[0][0], line2[0][1], line2[1][0], line2[1][1])
        dist3 = self.distancePointLine(line2[0][0], line2[0][1],
                                       line1[0][0], line1[0][1], line1[1][0], line1[1][1])
        dist4 = self.distancePointLine(line2[1][0], line2[1][1],
                                       line1[0][0], line1[0][1], line1[1][0], line1[1][1])

        return min(dist1, dist2, dist3, dist4)

    def merge_lines_segments1(self, lines, use_log=False):
        if (len(lines) == 1):
            return lines[0]

        line_i = lines[0]

        # orientation
        orientation_i = atan2((line_i[0][1] - line_i[1][1]), (line_i[0][0] - line_i[1][0]))

        points = []
        for line in lines:
            points.append(line[0])
            points.append(line[1])

        if (abs(degrees(orientation_i)) > 45) and abs(degrees(orientation_i)) < (90 + 45):

            # sort by y
            points = sorted(points, key=lambda point: point[1])

            if use_log:
                print("use y")
        else:

            # sort by x
            points = sorted(points, key=lambda point: point[0])

            if use_log:
                print("use x")

        return [points[0], points[len(points) - 1]]

    def merge_lines_pipeline_2(self, lines):
        super_lines_final = []
        super_lines = []

        for line in lines:
            create_new_group = True
            group_updated = False

            for group in super_lines:
                for line2 in group:
                    if self.get_distance(line2, line) < self.min_distance_to_merge:
                        # check the angle between lines
                        orientation_i = math.atan2((line[0][1] - line[1][1]), (line[0][0] - line[1][0]))
                        orientation_j = math.atan2((line2[0][1] - line2[1][1]), (line2[0][0] - line2[1][0]))

                        if int(abs(
                                math.degrees(orientation_i) - math.degrees(orientation_j))) < self.min_angle_to_merge:
                            group.append(line)

                            create_new_group = False
                            group_updated = True
                            break

                if group_updated:
                    break

            if create_new_group:
                new_group = []
                new_group.append(line)

                for idx, line2 in enumerate(lines):
                    # check the distance between lines
                    if self.get_distance(line2, line) < self.min_distance_to_merge:
                        # check the angle between lines
                        orientation_i = math.atan2((line[0][1] - line[1][1]), (line[0][0] - line[1][0]))
                        orientation_j = math.atan2((line2[0][1] - line2[1][1]), (line2[0][0] - line2[1][0]))

                        if int(abs(
                                math.degrees(orientation_i) - math.degrees(orientation_j))) < self.min_angle_to_merge:
                            new_group.append(line2)

                # append new group
                super_lines.append(new_group)

        for group in super_lines:
            super_lines_final.append(self.merge_lines_segments1(group))
        return super_lines_final

    def adjust_lines2(self):
        lines = self.lines
        del_rows = []
        parallel_thrld = 40
        for indx_j, line in enumerate(lines):
            line = line.reshape(2, 2, 1)
            orientation_j = self.get_orientation_deg(line)
            rel_orientation = 0
            cloud.th = 0
            if abs(cos(radians(orientation_j + cloud.th))) > cos(radians(parallel_thrld)):
                rel_orientation = np.array(
                    [0 - (orientation_j + cloud.th), 180 - (orientation_j + cloud.th),
                     -180 - (orientation_j + cloud.th)])
                rel_orientation = rel_orientation[abs(rel_orientation) == min(abs(rel_orientation))]
                self.lines[indx_j] = self.rotate_line(line, rel_orientation)
            elif abs(cos(radians(orientation_j + cloud.th))) < cos(radians(90 - parallel_thrld)):
                rel_orientation = np.array([(90 - (orientation_j + cloud.th)), -90 - (orientation_j + cloud.th)])
                rel_orientation = rel_orientation[abs(rel_orientation) == min(abs(rel_orientation))]
                self.lines[indx_j] = self.rotate_line(line, rel_orientation)
            else:
                del_rows.append(indx_j)
        self.lines = np.delete(self.lines, del_rows, 0)

    def adjust_lines(self):
        lines = self.lines
        delete_rows = []
        for indx_i, refernce_line in enumerate(lines):
            if indx_i in delete_rows:
                continue
            refernce_line = refernce_line.reshape(2, 2, 1)
            orientation_i = self.get_orientation_deg(refernce_line)
            group_parallel = [indx_i]
            group_normal = []
            for indx_j, line in enumerate(lines):
                if indx_j in delete_rows:
                    continue
                line = line.reshape(2, 2, 1)
                orientation_j = self.get_orientation_deg(line)
                rel_orientaion_deg = self.standardize_angle(orientation_i - orientation_j)
                if abs(cos(radians(rel_orientaion_deg))) > cos(radians(self.parallel_threshold)):
                    if indx_j in group_parallel:
                        continue
                    group_parallel.append(indx_j)
                elif abs(cos(radians(rel_orientaion_deg))) < cos(radians(90 - self.parallel_threshold)):
                    if indx_j in group_normal:
                        continue
                    group_normal.append(indx_j)
            self.parallel_group(group_parallel, lines)
            self.parallel_group(group_normal, lines)
            if group_normal:
                self.normal_group(group_normal, group_parallel, lines)
            delete_rows.extend(group_normal)
            delete_rows.extend(group_parallel)

    def parallel_group(self, group, lines):
        orientations_deg = []
        for line in lines[group]:
            angle = self.get_orientation_deg(line)
            orientations_deg.append(angle)
        x = np.cos(np.radians(orientations_deg)).mean()
        y = np.sin(np.radians(orientations_deg)).mean()
        mean_orientation = degrees(atan2(y, x))
        orientations_deg = np.array(orientations_deg)
        rel_orientaions_deg = mean_orientation - orientations_deg
        for indx, line in enumerate(lines[group]):
            self.lines[group[indx]] = self.rotate_line(line, self.standardize_angle(rel_orientaions_deg[indx]))

    def normal_group(self, group1, group2, lines):
        # mean_angle = np.mean([self.get_orientation_deg(lines[group1][0]), self.get_orientation_deg(lines[group2][0])+90])
        orientations_deg = []
        for line in lines[group1]:
            angle = self.get_orientation_deg(line)
            orientations_deg.append(angle)
        for line in lines[group2]:
            angle = self.get_orientation_deg(line) - 90
            orientations_deg.append(angle)

        x = np.cos(np.radians(orientations_deg)).mean()
        y = np.sin(np.radians(orientations_deg)).mean()
        mean_orientation = degrees(atan2(y, x))
        orientations_deg = np.array(orientations_deg)
        rel_orientaions_deg = mean_orientation - orientations_deg
        for indx, line in enumerate(lines[group1]):
            self.lines[group1[indx]] = self.rotate_line(line, self.standardize_angle(rel_orientaions_deg[indx]))
        for indx, line in enumerate(lines[group2]):
            self.lines[group2[indx]] = self.rotate_line(line, self.standardize_angle(
                rel_orientaions_deg[indx + len(group1) - 1]))

    def rotate_line(self, line, deg):
        x1, y1, x2, y2 = line.reshape(-1)
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        rad = radians(deg)
        # Rotate each around whole polyline's center point
        x1 = ((x1 - cx) * cos(rad) - (y1 - cy) * np.sin(rad)) + cx
        y1 = ((x1 - cx) * sin(rad) + (y1 - cy) * np.cos(rad)) + cy
        x2 = ((x2 - cx) * cos(rad) - (y2 - cy) * np.sin(rad)) + cx
        y2 = ((x2 - cx) * sin(rad) + (y2 - cy) * np.cos(rad)) + cy
        line_new = np.array([x1, y1, x2, y2])
        return line_new

    def standardize_angle(self, angle):
        # reduce the angle
        angle = angle % 360
        # force it to be the positive remainder, so that 0 <= angle < 360
        angle = (angle + 360) % 360
        # force into the minimum absolute value residue class, so that -180 < angle <= 180
        if angle > 180:
            angle -= 360
        return angle

    def get_pc(self):
        XYZ = []
        # scan_points = 360
        for line in self.lines:
            line = np.reshape(line, [-1])
            x1, y1, x2, y2 = line
            (x1, y1) = self.pixel2point(x1, y1)
            (x2, y2) = self.pixel2point(x2, y2)
            if np.size(XYZ) == 0:
                XYZ = np.linspace(np.array([x1, y1, 0]), np.array([x2, y2, 0]), 50)
            else:
                XYZ = np.vstack((XYZ, np.linspace(np.array([x1, y1, 0]), np.array([x2, y2, 0]), 50)))
        # sample_size = round(np.shape(XYZ)[0]/scan_points)
        # XYZ = np.array(XYZ)[::sample_size, :]
        # print(np.shape(XYZ))
        return XYZ

    def get_orientation_deg(self, line):
        line = line.reshape(2, 2, 1)
        orientation = math.atan2((line[0][1] - line[1][1]), (line[0][0] - line[1][0]))
        orientation = math.degrees(orientation)
        orientation = self.standardize_angle(orientation)
        return orientation

    def adjust_corners(self):
        points = np.reshape(self.lines, [-1, 2])
        corners = self.get_corners(self.lines)
        if corners.all():
            corners = np.reshape(corners, [-1, 2])
            for indx, point in enumerate(points):
                d = np.linalg.norm(corners - point, axis=1).reshape(corners.shape[0])
                if (d < self.corner_threshold).any():
                    points[indx] = self.corners[d < self.corner_threshold].reshape(-1)
        adjusted_corner_lines = np.reshape(points, [-1, 4])
        return adjusted_corner_lines

    def get_corners(self, lines):
        intersecs = []
        for i in range(np.shape(lines)[0]):
            line1 = lines[i]
            for j in range(np.shape(lines)[0]):
                if (abs(np.cos(self.get_orientation_deg(lines[j]) * np.pi / 180)) - abs(
                        np.cos(self.get_orientation_deg(line1) * np.pi / 180))) < 0.1:
                    break
                else:
                    line2 = lines[j]
                    point = self.line_intersection(line1, line2)
                    line1_min = min(np.linalg.norm(point - line1[:2]), np.linalg.norm(point - line1[2:4]))
                    line2_min = min(np.linalg.norm(point - line2[:2]), np.linalg.norm(point - line2[2:4]))
                    if (np.array([line2_min, line1_min]) < self.corner_threshold).all():
                        if np.size(intersecs) == 0:
                            intersecs = point
                        else:
                            intersecs = np.vstack((intersecs, point))
            self.corners = np.array(intersecs)
            print("corners:", self.corners)
            for point in self.corners:
                if np.shape(self.corners)[0] > 1:
                    self.corners = np.reshape(self.corners, [-1, 2])
                    d = np.linalg.norm(self.corners - point, axis=1)
                else:
                    d = np.linalg.norm(self.corners - point)
                self.corners[d < 0.1] = np.mean(self.corners[d < 0.1], axis=0)
                print("d", d)
        return self.corners

    def line_intersection(self, line1, line2):
        line1 = line1.reshape([2, 2])
        line2 = line2.reshape([2, 2])
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            raise Exception('lines do not intersect')
        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return np.array([x, y])


class Cloud:
    def __init__(self):
        self.cloud = PointCloud2()
        self.cloud_pub = rospy.Publisher('cloud_in_filtered', PointCloud2, queue_size=3)
        self.seq = 0
        self.points = []
        self.pc_filter = HoghmanFilter()
        self.xm = 0
        self.ym = 0
        self.th = 0
        self.pcd = o3d.geometry.PointCloud()

    def callback(self, msg):
        xyz = np.array([[0, 0, 0]])
        rgb = np.array([[0, 0, 0]])
        rospy.wait_for_service('point_cloud_transform')
        cloud_transform = rospy.ServiceProxy('point_cloud_transform', PointCloud2Request)
        try:
            msg = cloud_transform(msg, "map").cloud
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        gen = pc2.read_points(msg, skip_nans=True)
        int_data = list(gen)
        for x in int_data:
            test = x[3]
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f', test)
            i = struct.unpack('>l', s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)
            # prints r,g,b values in the 0-255 range
            # x,y,z can be retrieved from the x[0],x[1],x[2]
            for i in range(1):
                xyz = np.append(xyz, [[x[0], x[1], x[2] + i * 0.1]], axis=0)
                rgb = np.append(rgb, [[r, g, b]], axis=0)
        self.points = xyz
        self.points = self.remove_outlier(self.points)
        self.points, self.deg = self.sort_by_deg(self.points)
        self.pc_filter.set_cloud(self.points)
        self.points = self.pc_filter.get_pc()
        self.points = self.voxelize(self.points)
        rgbf = np.zeros([np.array(self.points).shape[0], 4])
        rgbf[:, [2, 3]] = 1
        self.cloud = self.point_cloud(np.hstack((np.array(self.points).tolist(), rgbf)), "map")
        rospy.wait_for_service('point_cloud_transform')
        cloud_transform = rospy.ServiceProxy('point_cloud_transform', PointCloud2Request)
        try:
            self.cloud = cloud_transform(self.cloud, "bot00_analyt").cloud
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        self.cloud.header.seq = self.seq
        self.cloud.header.stamp = rospy.Time.now()
        self.cloud_pub.publish(self.cloud)
        self.seq += 1

    def remove_outlier(self, points):
        self.pcd.points = o3d.utility.Vector3dVector(np.array(points))
        cl, ind = self.pcd.remove_radius_outlier(nb_points=5, radius=0.1)
        # cl, ind = pcd.remove_statistical_outlier(nb_neighbors=30,
        #                                          std_ratio=1)
        inlier_cloud = self.pcd.select_by_index(ind)
        return np.asarray(inlier_cloud.points)

    def set_mean_pose(self, msg):
        self.xm = msg.position.x
        self.ym = msg.position.y
        self.th = msg.position.z

    def sort_by_deg(self, points):
        x = points[:, 0] - self.xm
        y = points[:, 1] - self.ym
        th = np.arctan2(y, x)
        zipped_x = zip(th, x)
        zipped_y = zip(th, y)
        sorted_zipped_x = sorted(zipped_x)
        sorted_zipped_y = sorted(zipped_y)
        sorted_x = np.array([element for _, element in sorted_zipped_x])
        sorted_y = np.array([element for _, element in sorted_zipped_y])
        sorted_th = np.sort(th)
        points[:, 0] = (sorted_x + self.xm).tolist()
        points[:, 1] = (sorted_y + self.ym).tolist()
        return points, sorted_th

    def voxelize(self, points):
        sampled_points = []
        voxel_size = 0.03
        x_max, y_max, _ = np.max(points, axis=0)
        x_min, y_min, _ = np.min(points, axis=0)
        n_x = int((x_max-x_min)/voxel_size)
        n_y = int((y_max-y_min)/voxel_size)
        dx = (x_max - x_min) / n_x
        dy = (y_max - y_min) / n_x
        xv, yv = np.meshgrid(np.linspace(x_min, x_max, n_x), np.linspace(y_min, y_max, n_y))
        xv = xv.reshape(-1, 1)
        yv = yv.reshape(-1, 1)
        X = xv - points[:, 0].reshape(1, -1)
        Y = yv - points[:, 1].reshape(1, -1)
        d = (abs(X) < dx / 2) * (abs(Y) < dy / 2)
        a = np.nonzero(d)
        ocp_grids = np.unique(a[0])
        for grid in ocp_grids:
            if np.size(sampled_points) == 0:
                sampled_points = np.mean(points[a[1][a[0] == grid]], axis=0)
            else:
                sampled_points = np.vstack((sampled_points, np.mean(points[a[1][a[0] == grid]], axis=0)))
        return sampled_points

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


if __name__ == '__main__':
    rospy.init_node('pc_feature_extractor')
    cloud = Cloud()
    rospy.Subscriber("cloud_in", PointCloud2, cloud.callback)
    rospy.Subscriber("bot00_pose_offset", Pose, cloud.set_mean_pose)
    rate = rospy.Rate(10.0)
    rospy.spin()
