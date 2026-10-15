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


class SafeLQRTracker(Node):
    def __init__(self):
        super().__init__("lqr_tracker_safe")

        self.sub_plan = self.create_subscription(Path, "/plan", self.plan_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        self.plan_points = []

        # LQR反馈增益 [e_x, e_y, e_theta]
        self.K = np.array([
            [1.2, 0.0, 0.0],
            [0.0, 0.8, 1.5]
        ])
        self.v_des = 0.26          # 参考前进速度，与你小车参数匹配
        self.safe_distance = 0.40  # 安全阈值(m)，小于此距离紧急刹车
        self.min_obs_dist = 10.0

        self.get_logger().info("Safe‑LQR Tracker start. Laser emergency brake enabled.")

    def plan_cb(self, msg: Path):
        self.plan_points = []
        for ps in msg.poses:
            self.plan_points.append([ps.pose.position.x, ps.pose.position.y])

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

        # 紧急刹车逻辑
        if self.min_obs_dist < self.safe_distance:
            stop_cmd = Twist()
            self.pub_cmd.publish(stop_cmd)
            return

        idx = self.get_closest_point(rx, ry)
        if idx is None:
            self.pub_cmd.publish(Twist())
            return
        target_x, target_y = self.plan_points[idx]
        if idx + 1 >= len(self.plan_points):
            self.pub_cmd.publish(Twist())
            return

        next_x, next_y = self.plan_points[idx + 1]
        path_theta = np.arctan2(next_y - target_y, next_x - target_x)

        # 机器人本体坐标系下误差
        e_x = (target_x - rx) * np.cos(rth) + (target_y - ry) * np.sin(rth)
        e_y = -(target_x - rx) * np.sin(rth) + (target_y - ry) * np.cos(rth)
        e_th = wrap_angle(path_theta - rth)
        state = np.array([e_x, e_y, e_th])

        u = -self.K @ state
        vx = self.v_des + u[0]
        wz = u[1]

        # 速度限幅
        vx = np.clip(vx, -0.15, 0.26)
        wz = np.clip(wz, -0.8, 0.8)

        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SafeLQRTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
