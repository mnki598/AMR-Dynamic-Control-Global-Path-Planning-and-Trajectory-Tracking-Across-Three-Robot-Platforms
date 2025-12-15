# File: Mecanum_wheel Recorded Trajectory Tracking
# Description: Four Mecanum wheel dynamics and PD tracking controller
# Author: Mayank Pandey
# Date: 2025
# Note: Public demonstration version. Full implementation private.
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import math

class PathRecorder(Node):
    def __init__(self):
        super().__init__('record_path')
        self.declare_parameter('output_file', 'trajectory.csv')
        self.filename = self.get_parameter('output_file').value

        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time', 'x', 'y', 'theta'])

        self.start_time = self.get_clock().now()
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.get_logger().info(f"Recording started ? {self.filename}")

    def odom_callback(self, msg):
        t = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) - \
            (self.start_time.nanoseconds * 1e-9)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        self.writer.writerow([t, x, y, theta])

    def destroy_node(self):
        self.file.close()
        self.get_logger().info("Recording stopped, file saved.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
