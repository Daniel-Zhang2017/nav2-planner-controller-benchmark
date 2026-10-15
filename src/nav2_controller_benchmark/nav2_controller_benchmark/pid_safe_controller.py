import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data

def wrap_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

class SafePIDTracker(Node):
    def __init__(self):
        super().__init__("pid_tracker_safe")

        self.sub_plan = self.create_subscription(Path, "/plan", self.plan_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        self.plan_points = []
        # PID gains for cross-track error
        self.Kp = 1.8
        self.Ki = 0.02
        self.Kd = 0.35

        self.integral = 0.0
        self.last_cte = 0.0
        self.last_time = None

        self.v_des = 0.26
        self.safe_distance = 0.40
        self.min_obs_dist = 10.0

        self.get_logger().info("Safe-PID Tracker started, laser emergency brake enabled.")

    def plan_cb(self, msg: Path):
        self.plan_points = []
        for ps in msg.poses:
            self.plan_points.append([ps.pose.position.x, ps.pose.position.y])
        self.integral = 0.0
        self.last_cte = 0.0

    def scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges = ranges[np.isfinite(ranges)]
        if len(ranges) > 0:
            self.min_obs_dist = float(np.min(ranges))
        else:
            self.min_obs_dist = 10.0

    def get_closest_point(self, rx, ry):
        if len(self.plan_points) < 2:
            return None
        min_d = 1e9
        best_idx = 0
        for i, (x, y) in enumerate(self.plan_points):
            d = np.hypot(rx - x, ry - y)
            if d < min_d:
                min_d = d
                best_idx = i
        return best_idx

    def odom_cb(self, msg: Odometry):
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, rth = euler_from_quaternion([q.x, q.y, q.z, q.w])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Emergency brake
        if self.min_obs_dist < self.safe_distance:
            self.pub_cmd.publish(Twist())
            self.integral = 0.0
            return

        idx = self.get_closest_point(rx, ry)
        if idx is None or idx +1 >= len(self.plan_points):
            self.pub_cmd.publish(Twist())
            self.integral =0.0
            return

        target_x, target_y = self.plan_points[idx]
        next_x, next_y = self.plan_points[idx+1]
        path_theta = np.arctan2(next_y-target_y, next_x-target_x)

        # CTE in robot frame
        e_y = -(target_x - rx)*np.sin(rth) + (target_y - ry)*np.cos(rth)

        # PID calculation
        if self.last_time is None:
            dt = 0.0
        else:
            dt = stamp - self.last_time

        if dt > 1e-6:
            self.integral += e_y * dt
            deriv = (e_y - self.last_cte)/dt
        else:
            deriv =0.0

        wz = self.Kp * e_y + self.Ki * self.integral + self.Kd * deriv
        wz = np.clip(wz, -0.8, 0.8)

        cmd = Twist()
        cmd.linear.x = self.v_des
        cmd.angular.z = wz

        self.last_cte = e_y
        self.last_time = stamp
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SafePIDTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
