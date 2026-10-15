#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Benchmark evaluator: runs ONE trial and writes ONE CSV row.

The evaluator owns the trial lifecycle so every controller is measured
under identical conditions:

    1. reset the Gazebo robot to (0,0,0)            (/set_entity_state)
    2. publish the start pose to AMCL               (/initialpose)
    3. clear the costmaps                           (/.../clear_costmap)
    4. send the goal
         - mode=nav2    : NavigateToPose action     (DWB / MPPI / RPP)
         - mode=tracker : /benchmark/goal topic     (PID-Safe / LQR-Safe)
    5. record odometry / scan / cmd_vel / global plan until
       success | stuck | timeout | failure
    6. compute metrics (see metrics.py) and append a CSV row

Run (one trial):
    ros2 run nav2_controller_benchmark evaluator --ros-args \
        -p mode:=nav2 -p controller:=mppi -p scenario_id:=g1_detour \
        -p trial:=0 -p start_x:=0.0 -p start_y:=0.0 -p start_yaw:=0.0 \
        -p goal_x:=-8.0 -p goal_y:=-5.5 -p goal_yaw:=0.0 \
        -p output_csv:=/abs/path/run.csv -p timeout:=120.0
"""

import csv
import math
import os
import sys
import time as walltime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from nav2_controller_benchmark import metrics as M

CSV_COLUMNS = [
    'controller', 'scenario_id', 'trial', 'mode', 'success', 'reason',
    'completion_time_s', 'traveled_dist_m', 'plan_length_m', 'straight_line_m',
    'cte_mean_m', 'cte_max_m', 'cte_mean_initial_m', 'cte_max_initial_m',
    'min_clearance_all_m', 'min_clearance_forward_m',
    'jerk_lin_mean', 'jerk_lin_max', 'jerk_ang_mean', 'jerk_ang_max',
    'lin_oscillations', 'ang_oscillations',
    'avg_lin_vel', 'avg_ang_vel', 'collision',
]


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Evaluator(Node):
    def __init__(self):
        super().__init__('benchmark_evaluator')
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('mode', 'nav2')            # nav2 | tracker
        self.declare_parameter('controller', 'mppi')
        self.declare_parameter('scenario_id', 'g1_detour')
        self.declare_parameter('trial', 0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 2.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('output_csv', 'results/run.csv')
        self.declare_parameter('timeout', 120.0)
        self.declare_parameter('xy_goal_tolerance', 0.15)
        self.declare_parameter('yaw_goal_tolerance', 0.15)
        self.declare_parameter('robot_radius', 0.12)
        self.declare_parameter('entity_name', 'robot')
        self.declare_parameter('reset_pose', True)

        self.mode = str(self.get_parameter('mode').value)
        self.controller = str(self.get_parameter('controller').value)
        self.scenario_id = str(self.get_parameter('scenario_id').value)
        self.trial = int(self.get_parameter('trial').value)
        self.start = (float(self.get_parameter('start_x').value),
                      float(self.get_parameter('start_y').value),
                      float(self.get_parameter('start_yaw').value))
        self.goal = (float(self.get_parameter('goal_x').value),
                     float(self.get_parameter('goal_y').value),
                     float(self.get_parameter('goal_yaw').value))
        self.output_csv = str(self.get_parameter('output_csv').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.xy_tol = float(self.get_parameter('xy_goal_tolerance').value)
        self.yaw_tol = float(self.get_parameter('yaw_goal_tolerance').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.entity_name = str(self.get_parameter('entity_name').value)
        self.reset_pose = bool(self.get_parameter('reset_pose').value)

        # ---- subscriptions (samples are appended in callbacks) ----
        self.odom_samples = []       # (t, x, y, yaw) map frame
        self.odom_raw = []           # (t, x, y, yaw) odom frame
        self.scan_samples = []       # (t, min_all, min_forward)
        self.cmd_samples = []        # (t, vx, wz)
        self.plan_latest = None      # latest global plan (map frame)
        self.plan_initial = None     # first plan received after start
        self.tracker_status = None

        self._map = None             # OccupancyGrid -> distance transform
        self._dmap = None
        self._map_res = None
        self._map_origin = None
        self._map_shape = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_buffer.set_using_sim_time(True)

        self.create_subscription(Odometry, 'odom', self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/benchmark/tracker_status',
                                 self.on_status, 10)
        self.create_subscription(Path, '/benchmark/plan', self.on_plan, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(Path, '/planner_server/plan', self.on_plan, 10)
        self.create_subscription(Path, '/global_plan', self.on_plan, 10)
        self.create_subscription(OccupancyGrid, 'map', self.on_map,
                                 qos_profile_sensor_data)

        # NOTE: the /cmd_vel (Twist) subscription is created in on_init(),
        # called at the start of run_trial().

        # ---- services / actions ----
        self.entity_state_client = self.create_client(
            SetEntityState, '/set_entity_state')
        self.clear_global = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_costmap')
        self.clear_local = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_costmap')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # ---- publishers ----
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pub = self.create_publisher(
            self.goal_type(), '/benchmark/goal', 10)

    # ------------------------------------------------------------------
    @staticmethod
    def goal_type():
        from geometry_msgs.msg import PoseStamped
        return PoseStamped

    def on_init(self):
        """Fix the /cmd_vel subscription (Twist, not Odometry)."""
        from geometry_msgs.msg import Twist
        self._cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.on_cmd, 10)
    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q)
        self.odom_raw.append((self._stamp(msg), msg.pose.pose.position.x,
                              msg.pose.pose.position.y, yaw))
        # map-frame pose via tf (amcl) for CTE / collision
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return
        mx = t.transform.translation.x
        my = t.transform.translation.y
        myaw = quat_to_yaw(t.transform.rotation)
        self.odom_samples.append((self._stamp(msg), mx, my, myaw))
        if self._dmap is not None:
            self._check_collision(mx, my, self._stamp(msg))

    def on_scan(self, msg):
        amin, ainc = msg.angle_min, msg.angle_increment
        all_m, fwd_m = M.min_scan_clearance(
            msg.ranges, amin, ainc,
            forward_half_angle_deg=60.0,
            max_range=None)
        self.scan_samples.append((self._stamp(msg), all_m, fwd_m))

    def on_cmd(self, msg):
        self.cmd_samples.append((self._stamp(msg),
                                 float(msg.linear.x), float(msg.angular.z)))

    def on_status(self, msg):
        self.tracker_status = msg.data

    def on_plan(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not pts:
            return
        if self.plan_initial is None:
            self.plan_initial = pts
        self.plan_latest = pts

    def on_map(self, msg):
        if self._map is not None:
            return
        self._map = msg
        self._map_res = float(msg.info.resolution)
        self._map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        w, h = msg.info.width, msg.info.height
        arr = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
        occ = (arr > 65).astype(np.uint8)   # nav_msgs: >65 = occupied
        from scipy import ndimage
        self._dmap = ndimage.distance_transform_edt(1 - occ) * self._map_res
        self._map_shape = (h, w)
        self.get_logger().info('map distance transform ready')

    def _check_collision(self, mx, my, t):
        if self._dmap is None:
            return
        h, w = self._map_shape
        col = int(round((mx - self._map_origin[0]) / self._map_res))
        row = int(round((self._map_origin[1] + (h - 1) * self._map_res - my)
                        / self._map_res))
        if 0 <= row < h and 0 <= col < w:
            self._collision_hits += 1
            if self._dmap[row, col] < self.robot_radius:
                self._collision_samples += 1

    # ------------------------------------------------------------ utilities
    def _stamp(self, msg):
        s = msg.header.stamp
        return s.sec + s.nsec * 1e-9

    def now_sim(self):
        t = self.get_clock().now()
        return t.seconds_nanoseconds()[0] + t.seconds_nanoseconds()[1] * 1e-9

    def _wait_topic(self, msg_type, topic, timeout_s=15.0):
        got = []
        sub = self.create_subscription(msg_type, topic,
                                       lambda m: got.append(m), 10)
        t0 = walltime.time()
        while rclpy.ok() and not got and walltime.time() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return got[0] if got else None

    def _wait_tf(self, timeout_s=20.0):
        t0 = walltime.time()
        while rclpy.ok() and walltime.time() - t0 < timeout_s:
            try:
                self.tf_buffer.lookup_transform('map', 'base_footprint',
                                                rclpy.time.Time())
                return True
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException):
                rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _call_service(self, client, request, timeout_s=5.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            self.get_logger().warn(f'service {client.srv_name} unavailable')
            return False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        return future.result() is not None

    # ------------------------------------------------------------ trial steps
    def reset_robot(self):
        if not self.reset_pose:
            return
        req = SetEntityState.Request()
        req.state.name = self.entity_name
        req.state.pose.position.x = self.start[0]
        req.state.pose.position.y = self.start[1]
        req.state.pose.orientation.z = math.sin(self.start[2] / 2.0)
        req.state.pose.orientation.w = math.cos(self.start[2] / 2.0)
        req.state.twist.linear.x = 0.0
        req.state.twist.angular.z = 0.0
        req.state.reference_frame = 'world'
        self._call_service(self.entity_state_client, req)
        self.get_logger().info('robot pose reset')

    def set_initial_pose(self):
        for _ in range(2):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = self.start[0]
            msg.pose.pose.position.y = self.start[1]
            msg.pose.pose.orientation.z = math.sin(self.start[2] / 2.0)
            msg.pose.pose.orientation.w = math.cos(self.start[2] / 2.0)
            msg.pose.covariance[0] = 0.05
            msg.pose.covariance[7] = 0.05
            msg.pose.covariance[35] = 0.05
            self.initial_pose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            walltime.sleep(0.5)   # wall-clock: AMCL needs time to re-localise
        self.get_logger().info('initial pose published')

    def clear_costmaps(self):
        self._call_service(self.clear_global, ClearEntireCostmap.Request())
        if self.mode == 'nav2':
            self._call_service(self.clear_local, ClearEntireCostmap.Request())
        self.get_logger().info('costmaps cleared')

    def send_goal(self):
        if self.mode == 'tracker':
            msg = self.goal_type()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = self.goal[0]
            msg.pose.position.y = self.goal[1]
            msg.pose.orientation.z = math.sin(self.goal[2] / 2.0)
            msg.pose.orientation.w = math.cos(self.goal[2] / 2.0)
            self.goal_pub.publish(msg)
            self.get_logger().info(f'goal sent to tracker: {self.goal}')
            return None
        # nav2 mode: NavigateToPose action
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('navigate_to_pose server unavailable')
            return None
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal[0]
        goal.pose.pose.position.y = self.goal[1]
        goal.pose.pose.orientation.z = math.sin(self.goal[2] / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.goal[2] / 2.0)
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.result() or not future.result().accepted:
            self.get_logger().error('navigate_to_pose goal rejected')
            return None
        self.get_logger().info(f'goal sent to BT navigator: {self.goal}')
        return future.result()

    # ------------------------------------------------------------ trial loop
    def run_trial(self):
        self._collision_hits = 0
        self._collision_samples = 0
        self.on_init()

        self.get_logger().info(
            f'trial {self.controller}/{self.scenario_id}/#{self.trial} '
            f'start={self.start} goal={self.goal} mode={self.mode}')

        # 0. wait for the world
        if not self._wait_topic(OccupancyGrid, 'map', 20.0):
            self.get_logger().error('no /map — is map_server up?')
            return
        if not self._wait_topic(Odometry, 'odom', 20.0):
            self.get_logger().error('no /odom — is gazebo up?')
            return
        if not self._wait_topic(LaserScan, 'scan', 20.0):
            self.get_logger().error('no /scan — is the laser up?')
            return
        if not self._wait_tf(20.0):
            self.get_logger().error('no map->base_footprint tf (amcl?)')
            return

        # 1-3. identical starting conditions for every controller
        self.reset_robot()
        rclpy.spin_once(self, timeout_sec=0.5)
        self.set_initial_pose()
        self.clear_costmaps()
        rclpy.spin_once(self, timeout_sec=0.5)

        # 4. go
        t0 = self.now_sim()
        goal_handle = self.send_goal()
        if goal_handle is None and self.mode == 'nav2':
            return
        if self.mode == 'tracker':
            # make sure the tracker has a path before measuring
            while rclpy.ok() and self.plan_initial is None:
                if self.now_sim() - t0 > 20.0:
                    self.get_logger().error('tracker produced no plan in 20s')
                    break
                rclpy.spin_once(self, timeout_sec=0.1)
            t0 = self.now_sim()   # measure from plan availability

        # 5. monitor
        reason = 'timeout'
        success = 0
        goal_reached_streak = 0
        result_future = None
        if self.mode == 'nav2' and goal_handle is not None:
            result_future = goal_handle.get_result_async()

        while rclpy.ok():
            elapsed = self.now_sim() - t0
            if elapsed > self.timeout:
                reason = 'timeout'
                if self.mode == 'nav2' and goal_handle is not None:
                    self.nav_client.cancel_goal_async(goal_handle)
                break
            rclpy.spin_once(self, timeout_sec=0.02)

            if self.mode == 'nav2':
                if result_future is not None and result_future.done():
                    res = result_future.result()
                    status = res.status
                    if status == 4:            # SUCCEEDED
                        success, reason = 1, 'success'
                    elif status in (2, 3, 5):  # ABORTED/CANCELED
                        reason = f'nav2_status_{status}'
                    else:
                        reason = f'nav2_status_{status}'
                    break
            else:
                if self.tracker_status in ('goal_reached',):
                    success, reason = 1, 'success'
                    break
                if self.tracker_status in ('stuck', 'failed'):
                    reason = self.tracker_status
                    break
                # independent odometry-based goal check (belt and braces)
                if self.odom_samples:
                    _, x, y, yaw = self.odom_samples[-1]
                    dx, dy = self.goal[0] - x, self.goal[1] - y
                    if (math.hypot(dx, dy) <= self.xy_tol
                            and abs(M.wrap_angle(self.goal[2] - yaw))
                            <= self.yaw_tol):
                        goal_reached_streak += 1
                        if goal_reached_streak >= 3:
                            success, reason = 1, 'success'
                            break
                    else:
                        goal_reached_streak = 0

        self.get_logger().info(f'trial done: reason={reason} success={success}')

        # 6. metrics + CSV row
        row = self.compute_row(success, reason, t0)
        self.write_row(row)
        self.get_logger().info(f'CSV row written: {self.output_csv}')

    # ------------------------------------------------------------ metrics
    def compute_row(self, success, reason, t0):
        row = dict.fromkeys(CSV_COLUMNS, float('nan'))
        row.update({'controller': self.controller, 'scenario_id': self.scenario_id,
                    'trial': self.trial, 'mode': self.mode,
                    'success': success, 'reason': reason})

        if self.odom_samples:
            ts = np.array([s[0] for s in self.odom_samples])
            xy = np.array([[s[1], s[2]] for s in self.odom_samples])
            yaws = np.array([s[3] for s in self.odom_samples])
            t_fin = ts[-1]
            row['completion_time_s'] = round(float(t_fin - t0), 3)
            row['traveled_dist_m'] = round(M.traveled_length(xy[:, 0], xy[:, 1]), 3)

            ref = self.plan_latest if self.plan_latest else None
            if ref:
                cte = M.cross_track_errors(xy, ref)
                row['cte_mean_m'] = round(float(np.nanmean(cte)), 4)
                row['cte_max_m'] = round(float(np.nanmax(cte)), 4)
            ref0 = self.plan_initial if self.plan_initial else None
            if ref0:
                cte0 = M.cross_track_errors(xy, ref0)
                row['cte_mean_initial_m'] = round(float(np.nanmean(cte0)), 4)
                row['cte_max_initial_m'] = round(float(np.nanmax(cte0)), 4)
            if self.plan_initial:
                row['plan_length_m'] = round(M.path_length(
                    np.asarray(self.plan_initial, dtype=float)), 3)
            row['straight_line_m'] = round(
                math.hypot(self.goal[0] - self.start[0],
                           self.goal[1] - self.start[1]), 3)
            if self._collision_hits:
                row['collision'] = round(self._collision_samples
                                         / self._collision_hits, 4)

        if self.scan_samples:
            alls = [s[1] for s in self.scan_samples if np.isfinite(s[1])]
            fwds = [s[2] for s in self.scan_samples if np.isfinite(s[2])]
            if alls:
                row['min_clearance_all_m'] = round(float(np.min(alls)), 3)
            if fwds:
                row['min_clearance_forward_m'] = round(float(np.min(fwds)), 3)

        if self.cmd_samples:
            ct = np.array([s[0] for s in self.cmd_samples])
            vx = np.array([s[1] for s in self.cmd_samples])
            wz = np.array([s[2] for s in self.cmd_samples])
            row['jerk_lin_mean'], row['jerk_lin_max'] = M.jerk_stats(ct, vx)
            row['jerk_ang_mean'], row['jerk_ang_max'] = M.jerk_stats(ct, wz)
            row['lin_oscillations'] = M.oscillation_count(vx)
            row['ang_oscillations'] = M.oscillation_count(wz)
            row['avg_lin_vel'] = round(float(np.mean(np.abs(vx))), 4)
            row['avg_ang_vel'] = round(float(np.mean(np.abs(wz))), 4)

        # round float cells, keep strings
        return row

    def write_row(self, row):
        path = self.output_csv
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        write_header = not os.path.exists(path) or os.path.getsize(path) == 0
        with open(path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
            if write_header:
                writer.writeheader()
            writer.writerow(row)


def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
    try:
        node.run_trial()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#（注：内容由AI生成）
