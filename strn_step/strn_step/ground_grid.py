
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
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

        # 前方 0.6 ～ 3.6 m
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
        # 高さマップ（従来の形式）
        # =========================

        self.height_publisher = self.create_publisher(
            PointCloud2,
            '/height_grid',
            qos
        )

        # =========================
        # 高さ統計マップ（新規追加）
        # =========================

        self.stats_publisher = self.create_publisher(
            PointCloud2,
            '/height_grid_stats',
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
        # 各格子の高さ・統計量を計算
        # =========================

        height_points = []
        stats_points = []

        for (ix, iy), z_values in grid.items():

            if len(z_values) < self.min_points:
                continue

            # =====================
            # 高さの中央値（従来）
            # =====================

            height = float(
                np.median(z_values)
            )

            # =====================
            # 高さの統計量（新規追加）
            # =====================

            # 高さの分散（単位 m^2）
            variance = float(
                np.var(z_values)
            )

            # 高さの標準偏差（単位 m）
            std_dev = float(
                np.std(z_values)
            )

            # グリッド内の点数
            point_count = len(z_values)

            # =====================
            # 格子中心座標
            # =====================

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

            # =====================
            # 従来の高さマップ
            # =====================

            height_points.append(
                (
                    center_x,
                    center_y,
                    height
                )
            )

            # =====================
            # 新しい高さ統計マップ
            # =====================

            stats_points.append(
                (
                    center_x,
                    center_y,
                    height,
                    variance,
                    std_dev,
                    float(point_count)
                )
            )

        # =========================
        # 高さマップをPublish（従来）
        # =========================

        height_msg = point_cloud2.create_cloud_xyz32(
            msg.header,
            height_points
        )

        self.height_publisher.publish(
            height_msg
        )

        # =========================
        # 高さ統計マップをPublish（新規）
        # =========================

        fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='variance',
                offset=12,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='std_dev',
                offset=16,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='point_count',
                offset=20,
                datatype=PointField.FLOAT32,
                count=1
            )
        ]

        stats_msg = point_cloud2.create_cloud(
            msg.header,
            fields,
            stats_points
        )

        self.stats_publisher.publish(
            stats_msg
        )

        # =========================
        # ログ表示
        # =========================

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

