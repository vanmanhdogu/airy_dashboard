#!/usr/bin/env python3
"""Publish the Airy's internal temperatures, read from the MSOP packet header.

Subscribes /rslidar_packets (rslidar_msg/RslidarPacket, needs send_packet_ros: true)
and publishes at 1 Hz (sensor_msgs/Temperature, degC):
    /airy/temperature           main board   (header bytes 38-39)
    /airy/temperature_topboard  top board    (header bytes 40-41)
Encoding as rs_driver parseTempInLe(): 5-bit lsb in byte0[7:3], sign in byte1[7],
7-bit msb in byte1[6:0]; resolution 0.0625 degC.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rslidar_msg.msg import RslidarPacket
from sensor_msgs.msg import Temperature


def temp_le(b0, b1):
    t = ((b1 & 0x7F) << 5) + (b0 >> 3)
    return (-t if b1 & 0x80 else t) * 0.0625


class AiryTemperature(Node):
    def __init__(self):
        super().__init__('airy_temperature')
        self.main = self.create_publisher(Temperature, '/airy/temperature', 10)
        self.top = self.create_publisher(Temperature, '/airy/temperature_topboard', 10)
        self.latest = None
        self.create_subscription(RslidarPacket, '/rslidar_packets', self.cb,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self.publish)
        # Optional CSV log: ros2 param `csv` (path); also read by run_and_record.sh progress.
        path = self.declare_parameter('csv', '').value
        self.csv = open(path, 'a', buffering=1) if path else None
        if self.csv and self.csv.tell() == 0:
            self.csv.write('unix_time,main_degC,topboard_degC\n')

    def cb(self, msg):
        d = msg.data
        if msg.is_difop or len(d) < 42:
            return
        self.latest = (msg.header, temp_le(d[38], d[39]), temp_le(d[40], d[41]))

    def publish(self):
        if self.latest is None:
            self.get_logger().warn('no MSOP packets on /rslidar_packets yet',
                                   throttle_duration_sec=5.0)
            return
        header, t_main, t_top = self.latest
        for pub, t in ((self.main, t_main), (self.top, t_top)):
            m = Temperature()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = header.frame_id
            m.temperature = t
            pub.publish(m)
        if self.csv:
            now = self.get_clock().now().nanoseconds * 1e-9
            self.csv.write(f'{now:.3f},{t_main:.4f},{t_top:.4f}\n')
        self.get_logger().info(f'main {t_main:.2f} C   top board {t_top:.2f} C',
                               throttle_duration_sec=60.0)


def main():
    rclpy.init()
    rclpy.spin(AiryTemperature())


if __name__ == '__main__':
    main()
