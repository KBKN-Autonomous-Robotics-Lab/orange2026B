import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy
)

import numpy as np


class HeightGrid(Node):

    def __init__(self):

        super().__init__('height_grid')

        # =========================
        # 処理範囲
        # =========================

        # 前方 0.6 ～ 3.6 m    tyousei 
        self.x_min = 0.6
        self.x_max = 3.6

        # 左右 ±1 m
        self.y_min = -1.0
        self.y_max = 1.0

        # 10 cm × 10 cm
        self.grid_size = 0.1

        # 1マスに最低必要な点数
        self.min_points = 3

        # =========================
        # QoS
        # =========================

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # =========================
        # /pcd_rotation を受信
        # =========================

        self.subscription = self.create_subscription(
            PointCloud2,
            '/pcd_rotation',
            self.pointcloud_callback,
            qos
        )

        # =========================
        # 前方3mの点群
        # =========================

        self.front_publisher = self.create_publisher(
            PointCloud2,
            '/front_pointcloud',
            qos
        )

        # =========================
        # 高さマップ
        # =========================

        self.height_publisher = self.create_publisher(
            PointCloud2,
            '/height_grid',
            qos
        )

        self.get_logger().info(
            'Height grid node started'
        )


    def pointcloud_callback(self, msg):

        # =========================
        # PointCloud2 → xyz
        # =========================

        points = point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        )

        front_points = []

        # key   = (ix, iy)
        # value = その格子内のz一覧
        grid = {}

        # =========================
        # 前方3mだけ抽出
        # =========================

        for point in points:

            x = float(point[0])
            y = float(point[1])
            z = float(point[2])

            if not (
                self.x_min <= x < self.x_max and
                self.y_min <= y < self.y_max
            ):
                continue

            front_points.append(
                (x, y, z)
            )

            # =====================
            # 格子番号
            # =====================

            ix = int(
                (x - self.x_min)
                / self.grid_size
            )

            iy = int(
                (y - self.y_min)
                / self.grid_size
            )

            key = (ix, iy)

            if key not in grid:
                grid[key] = []

            grid[key].append(z)

        # =========================
        # 前方点群をPublish
        # =========================

        front_msg = point_cloud2.create_cloud_xyz32(
            msg.header,
            front_points
        )

        self.front_publisher.publish(
            front_msg
        )

        # =========================
        # 各格子の高さを計算
        # =========================

        height_points = []

        for (ix, iy), z_values in grid.items():

            if len(z_values) < self.min_points:
                continue

            # 外れ値の影響を受けにくい中央値
            height = float(
                np.median(z_values)
            )

            # 格子中心座標
            center_x = (
                self.x_min
                + (ix + 0.5)
                * self.grid_size
            )

            center_y = (
                self.y_min
                + (iy + 0.5)
                * self.grid_size
            )

            height_points.append(
                (
                    center_x,
                    center_y,
                    height
                )
            )

        # =========================
        # 高さマップをPublish
        # =========================

        height_msg = point_cloud2.create_cloud_xyz32(
            msg.header,
            height_points
        )

        self.height_publisher.publish(
            height_msg
        )

        self.get_logger().info(
            f'Front points: {len(front_points)}, '
            f'Valid grid cells: {len(height_points)}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = HeightGrid()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
