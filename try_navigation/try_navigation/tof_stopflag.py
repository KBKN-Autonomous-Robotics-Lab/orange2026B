import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial


class ToFSerialNode(Node):

    def __init__(self):
        super().__init__('tof_serial_node')

        self.publisher = self.create_publisher(
            Bool,
            'tof_flag',
            10
        )

        self.ser = serial.Serial(
            '/dev/sensors/tof',
            115200,
            timeout=0.1
        )

        self.timer = self.create_timer(0.05, self.read_serial)

        self.get_logger().info('ToF serial node started')

    def read_serial(self):

        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode(
                'utf-8',
                errors='ignore'
            ).strip()

            if data == "STOP":
                msg = Bool()
                msg.data = True
                self.publisher.publish(msg)

                self.get_logger().info('STOP')
            
            elif data == "GO":
                msg = Bool()
                msg.data = False
                self.publisher.publish(msg)
                self.get_logger().info('GO')


def main(args=None):
    rclpy.init(args=args)

    node = ToFSerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

