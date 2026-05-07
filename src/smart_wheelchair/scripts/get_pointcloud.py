#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
深度图转三维点云节点（适配 Gazebo RGB-D 摄像头）

坐标系说明：
  - camera_link (URDF): X=前, Y=左, Z=上
  - 相机光学坐标系:     X=右, Y=下, Z=前（深度）

常见陷阱（IMPORTANT）：
  标准针孔反投影公式 (X,Y,Z) = ((u-cx)*Z/fx, (v-cy)*Z/fy, depth)
  输出的是"相机光学坐标系"，其中 Z 轴是光轴（向前）。
  但 URDF 中的 camera_link 坐标轴和 base_link 对齐：X=前, Y=左, Z=上。
  因此必须把光学坐标系的 (X_opt, Y_opt, Z_opt) 映射到 camera_link：
    depth (Z_opt)  → camera_link X（前方）
    (u-cx)*depth/fx (X_opt) → camera_link -Y（右方）
    (v-cy)*depth/fy (Y_opt) → camera_link -Z（下方）
  如果直接把 depth 填到 Z，点云会出现在 camera_link 的正上方。
"""

import rospy
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pcl2

FIELDS = [
    PointField('x',  0, PointField.FLOAT32, 1),
    PointField('y',  4, PointField.FLOAT32, 1),
    PointField('z',  8, PointField.FLOAT32, 1),
]


class Depth2PointCloud:
    def __init__(self):
        rospy.init_node('depth_to_pointcloud')
        self.bridge = CvBridge()

        self.depth = None
        self.info = None
        self.encoding = None

        self.sub_depth = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.cb_depth
        )
        self.sub_info = rospy.Subscriber(
            "/camera/depth/camera_info", CameraInfo, self.cb_info
        )

        self.pub = rospy.Publisher("/pointcloud_output", PointCloud2, queue_size=1)
        rospy.loginfo("深度图转点云节点已启动，等待 /camera/depth/image_raw ...")

    def cb_depth(self, msg):
        try:
            if msg.encoding == "16UC1":
                self.depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
                self.encoding = "16UC1"
            elif msg.encoding == "32FC1":
                self.depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
                self.encoding = "32FC1"
            else:
                rospy.logwarn_throttle(5.0, f"不支持的深度图编码: {msg.encoding}")
        except Exception as e:
            rospy.logerr(f"深度图转换失败: {e}")

    def cb_info(self, msg):
        self.info = msg

    @staticmethod
    def depth_to_pointcloud(depth, info, step=4):
        """
        将深度图转换为 camera_link 坐标系下的点云。
        camera_link: X=前, Y=左, Z=上
        """
        h, w = depth.shape
        fx, fy = info.K[0], info.K[4]
        cx, cy = info.K[2], info.K[5]

        # 降采样网格
        u = np.arange(0, w, step)
        v = np.arange(0, h, step)
        u, v = np.meshgrid(u, v)

        # 提取深度值
        d = depth[v, u]

        # 统一单位为米
        if d.dtype == np.uint16:
            depth_m = d.astype(np.float32) / 1000.0
        else:
            depth_m = d.astype(np.float32)

        # 过滤无效深度
        mask = (depth_m > 0.05) & (depth_m < 10.0)
        u = u[mask]
        v = v[mask]
        depth_m = depth_m[mask]

        # 标准针孔反投影（光学坐标系）
        x_opt = (u - cx) * depth_m / fx   # 右方
        y_opt = (v - cy) * depth_m / fy   # 下方
        z_opt = depth_m                    # 前方（深度）

        # 转换到 camera_link 坐标系（X=前, Y=左, Z=上）
        x_link = z_opt           # 深度 → 前方
        y_link = -x_opt          # 右方 → 负 Y（即右方）
        z_link = -y_opt          # 下方 → 负 Z（即下方）

        points = np.stack((x_link, y_link, z_link), axis=-1).astype(np.float32)
        return points.tolist()

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.depth is None or self.info is None:
                rate.sleep()
                continue

            points = self.depth_to_pointcloud(self.depth, self.info, step=4)
            rospy.loginfo_throttle(1.0, f"生成点云数量：{len(points)}，编码：{self.encoding}")

            if len(points) > 0:
                header = rospy.Header()
                header.stamp = rospy.Time.now()
                header.frame_id = "camera_link"

                cloud = pcl2.create_cloud(header, FIELDS, points)
                self.pub.publish(cloud)

            rate.sleep()


if __name__ == '__main__':
    try:
        Depth2PointCloud().run()
    except rospy.ROSInterruptException:
        pass
