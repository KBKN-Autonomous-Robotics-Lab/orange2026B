import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data


class WheelStepDetector(Node):

    def __init__(self):
        super().__init__('wheel_step_detector')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/height_grid',
            self.height_grid_callback,
            qos_profile_sensor_data
        )

        # =========================
        # Parameters
        # =========================

        # 段差検知する前方距離 [m]
        self.detection_range = 2.0

        # 段差高さ閾値 [m]
        self.step_threshold = 0.04

        # STOP判定距離 [m]
        self.stop_distance = 1.2

        # 左右車輪中心位置 [m]
        self.left_wheel_y = 0.175
        self.right_wheel_y = -0.175

        # 車輪軌跡として確認する幅
        # 中心から±10 cm
        self.track_half_width = 0.10

        # 段差後に確認するセル数
        self.confirm_cells = 2

        self.get_logger().info(
            'Wheel step detector started'
        )

    def height_grid_callback(self, msg):

        # PointCloud2からx, y, zを取得
        points = point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        )

        left_cells = []
        right_cells = []

        # =========================
        # 左右車輪軌跡上のセル抽出
        # =========================

        for point in points:

            x = float(point[0])
            y = float(point[1])
            z = float(point[2])

            # 前方2m以内のみ使用
            if x <= 0.0 or x > self.detection_range:
                continue

            left_min_y = (
                self.left_wheel_y
                - self.track_half_width
            )

            left_max_y = (
                self.left_wheel_y
                + self.track_half_width
            )

            right_min_y = (
                self.right_wheel_y
                - self.track_half_width
            )

            right_max_y = (
                self.right_wheel_y
                + self.track_half_width
            )

            # 左車輪軌跡
            if left_min_y <= y <= left_max_y:
                left_cells.append((x, y, z))

            # 右車輪軌跡
            if right_min_y <= y <= right_max_y:
                right_cells.append((x, y, z))

        # =========================
        # x方向に近い順で並べる
        # =========================

        left_cells.sort(key=lambda p: p[0])
        right_cells.sort(key=lambda p: p[0])

        # =========================
        # セル数表示
        # =========================

        self.get_logger().info(
            f'LEFT cells={len(left_cells)}, '
            f'RIGHT cells={len(right_cells)}'
        )

        # =========================
        # LEFT軌跡を全部表示
        # =========================

        if left_cells:

            self.get_logger().info(
                'LEFT trajectory:'
            )

            for x, y, z in left_cells:
                self.get_logger().info(
                    f'LEFT '
                    f'x={x:.2f}, '
                    f'y={y:.2f}, '
                    f'z={z:.3f}'
                )

        # =========================
        # RIGHT軌跡を全部表示
        # =========================

        if right_cells:

            self.get_logger().info(
                'RIGHT trajectory:'
            )

            for x, y, z in right_cells:
                self.get_logger().info(
                    f'RIGHT '
                    f'x={x:.2f}, '
                    f'y={y:.2f}, '
                    f'z={z:.3f}'
                )

        # =========================
        # 段差判定
        # =========================

        left_step = self.detect_step(
            left_cells,
            'LEFT'
        )

        right_step = self.detect_step(
            right_cells,
            'RIGHT'
        )

        # =========================
        # STOP判定
        # =========================

        stop = False

        if left_step is not None:
            if left_step['distance'] <= self.stop_distance:
                stop = True

        if right_step is not None:
            if right_step['distance'] <= self.stop_distance:
                stop = True

        if stop:

            self.get_logger().warn(
                '========== STOP =========='
            )

        else:

            self.get_logger().info(
                'SAFE'
            )

    def detect_step(self, cells, side):

        # 基準セル
        # + 段差セル
        # + 確認2セル
        # が必要
        if len(cells) < self.confirm_cells + 2:
            return None

        max_index = (
            len(cells)
            - self.confirm_cells
            - 1
        )

        for i in range(max_index):

            x1, y1, z1 = cells[i]
            x2, y2, z2 = cells[i + 1]

            # 高さ差
            delta_h = z2 - z1

            # 高さ差も確認用に表示
            self.get_logger().info(
                f'{side} '
                f'{x1:.2f} -> {x2:.2f} m, '
                f'delta_h={delta_h:.3f} m'
            )

            # 4cm未満なら無視
            if abs(delta_h) < self.step_threshold:
                continue

            # =========================
            # 段差候補
            # =========================

            self.get_logger().warn(
                f'{side} STEP candidate: '
                f'distance={x2:.2f} m, '
                f'delta_h={delta_h:.3f} m'
            )

            confirmed = True

            # =========================
            # その先2セルを確認
            # =========================

            for j in range(
                1,
                self.confirm_cells + 1
            ):

                _, _, z_after = (
                    cells[i + 1 + j]
                )

                # 上り段差
                if delta_h > 0:

                    if z_after < (
                        z1
                        + self.step_threshold
                    ):

                        confirmed = False
                        break

                # 下り段差
                else:

                    if z_after > (
                        z1
                        - self.step_threshold
                    ):

                        confirmed = False
                        break

            # =========================
            # 段差確定
            # =========================

            if confirmed:

                self.get_logger().warn(
                    f'{side} STEP DETECTED: '
                    f'distance={x2:.2f} m, '
                    f'delta_h={delta_h:.3f} m'
                )

                return {
                    'distance': x2,
                    'height_diff': delta_h
                }

            else:

                self.get_logger().info(
                    f'{side} candidate rejected'
                )

        return None


def main(args=None):

    rclpy.init(args=args)

    node = WheelStepDetector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        # Ctrl+Cでshutdown済みの場合のエラー防止
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
