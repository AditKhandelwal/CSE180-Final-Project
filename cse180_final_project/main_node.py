import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid


class MainNode(Node):
    def __init__(self):
        super().__init__('cse180_final_main')

        # Latest data holders
        self.robot_pose = None        # (x, y, yaw)
        self.latest_scan = None       # LaserScan
        self.map = None               # OccupancyGrid

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Timer to periodically log status
        self.status_timer = self.create_timer(2.0, self.status_timer_callback)

        self.get_logger().info('MainNode initialized: subscribed to /amcl_pose, /scan, /map')

    # Callbacks

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        # Extract x, y, yaw from pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        # yaw (z-rotation) from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.robot_pose = (x, y, yaw)

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def map_callback(self, msg: OccupancyGrid):
        # Only store the map once
        if self.map is None:
            self.map = msg
            self.get_logger().info(
                f"Received map: {msg.info.width}x{msg.info.height}, "
                f"resolution={msg.info.resolution}"
            )

    def status_timer_callback(self):
        # Periodic debug print
        if self.robot_pose is not None:
            x, y, yaw = self.robot_pose
            self.get_logger().info(
                f"Robot pose (amcl): x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad"
            )
        else:
            self.get_logger().warn("No /amcl_pose received yet")

        if self.latest_scan is not None:
            n = len(self.latest_scan.ranges)
            self.get_logger().info(f"Latest /scan has {n} ranges")
        else:
            self.get_logger().warn("No /scan received yet")

        if self.map is None:
            self.get_logger().warn("No /map received yet")


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
