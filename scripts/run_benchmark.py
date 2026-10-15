#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run the full controller benchmark automatically.

Orchestration (must be run in a sourced ROS 2 Humble environment):

    controller x scenario x trial
        Gazebo (once)  ->  Nav2 stack for the controller  ->  evaluator per trial
        evaluator writes one row per trial into results/run.csv
    -> optional: analyze_results.py produces the summary tables + plots

Usage examples:

    # all 5 controllers, 3 trials each, all scenarios, headless
    python3 scripts/run_benchmark.py --controllers mppi,dwb,rpp,pid_safe,lqr_safe \
        --trials 3

    # just DWB + MPPI on the two detour scenarios, 5 trials, keep sim running
    python3 scripts/run_benchmark.py --controllers dwb,mppi \
        --scenario-ids g1_detour,g2_detour --trials 5 --no-restart-sim

    # stack already running manually (only the evaluator loop)
    python3 scripts/run_benchmark.py --no-stack --controllers rpp --trials 1

    # and then:
    python3 scripts/analyze_results.py --csv results/run.csv
"""

import argparse
import datetime
import os
import signal
import subprocess
import sys
import time
import yaml
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

NAV2_CONTROLLERS = ('mppi', 'dwb', 'rpp')
TRACKER_CONTROLLERS = ('pid_safe', 'lqr_safe')
ALL_CONTROLLERS = NAV2_CONTROLLERS + TRACKER_CONTROLLERS


def sh(*cmd, log=None, **kw):
    """Run a shell command, tee stdout+stderr to `log` (Path or None)."""
    kw.setdefault('shell', True)
    if log is not None:
        log = open(log, 'a')
        kw['stdout'] = log
        kw['stderr'] = subprocess.STDOUT
    return subprocess.run(cmd, **kw)


def spawn(*cmd, log=None):
    """Start a background process group; returns Popen."""
    logf = open(log, 'a') if log else None
    return subprocess.Popen(cmd, shell=True, stdout=logf or subprocess.DEVNULL,
                            stderr=subprocess.STDOUT, start_new_session=True)


def stop(proc, name, grace=10.0):
    """SIGINT the process group, then SIGKILL after grace."""
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except (ProcessLookupError, PermissionError):
        pass
    try:
        proc.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass


# ---------------------------------------------------------------- readiness
def wait_for_topic(node, msg_type, topic, timeout=60.0):
    got = []
    sub = node.create_subscription(msg_type, topic, got.append, 10)
    t0 = time.time()
    while time.time() - t0 < timeout:
        from rclpy import spin_once
        spin_once(node, timeout_sec=0.2)
        if got:
            node.destroy_subscription(sub)
            return True
    node.destroy_subscription(sub)
    return False


def wait_for_service(node, srv_type, name, timeout=60.0):
    cli = node.create_client(srv_type, name)
    ok = cli.wait_for_service(timeout_sec=timeout)
    node.destroy_client(cli)
    return ok


def wait_for_action(node, action_type, name, timeout=60.0):
    from rclpy.action import ActionClient
    ac = ActionClient(node, action_type, name)
    ok = ac.wait_for_server(timeout_sec=timeout)
    ac.destroy()
    return ok


# ------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--controllers', default=','.join(ALL_CONTROLLERS),
                    help='comma-separated controller list')
    ap.add_argument('--trials', type=int, default=3, help='trials per scenario')
    ap.add_argument('--scenarios', default=str(REPO_ROOT / 'src' / 'robot_navigation2'
                                               / 'config' / 'benchmark' / 'scenarios.yaml'),
                    help='scenario yaml path')
    ap.add_argument('--scenario-ids', default='',
                    help='comma-separated subset of scenario ids (default: all)')
    ap.add_argument('--timeout', type=float, default=120.0, help='per-trial timeout (s)')
    ap.add_argument('--output-dir', default='',
                    help='results dir (default: results/run_<timestamp>)')
    ap.add_argument('--rviz', action='store_true', default=False,
                    help='show RViz (default: headless benchmark)')
    ap.add_argument('--no-restart-sim', action='store_true', default=False,
                    help='keep Gazebo alive across controllers (faster)')
    ap.add_argument('--no-stack', action='store_true', default=False,
                    help='assume Gazebo+Nav2 already running; only loop the evaluator')
    ap.add_argument('--no-analyze', action='store_true', default=False,
                    help='skip analyze_results.py at the end')
    args = ap.parse_args()

    controllers = [c.strip() for c in args.controllers.split(',') if c.strip()]
    for c in controllers:
        if c not in ALL_CONTROLLERS:
            sys.exit(f'unknown controller: {c} (choose from {ALL_CONTROLLERS})')

    scenarios = yaml.safe_load(Path(args.scenarios).read_text())['scenarios']
    if args.scenario_ids:
        wanted = set(args.scenario_ids.split(','))
        scenarios = [s for s in scenarios if s['id'] in wanted]
    if not scenarios:
        sys.exit('no scenarios selected')

    out_dir = Path(args.output_dir) if args.output_dir else \
        REPO_ROOT / 'results' / f"run_{datetime.datetime.now():%Y%m%d_%H%M%S}"
    (out_dir / 'logs').mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / 'run.csv'
    log_dir = out_dir / 'logs'
    print(f'output dir : {out_dir}')
    print(f'controllers: {controllers}')
    print(f'scenarios  : {[s["id"] for s in scenarios]}')
    print(f'trials     : {args.trials}')

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node('benchmark_orchestrator')

    gazebo = None
    try:
        if not args.no_stack:
            gazebo = spawn(
                f'ros2 launch robot_description gazebo_robot.launch.py',
                log=log_dir / 'gazebo.log')
            print('waiting for Gazebo (/scan, /odom)...')
            if not (wait_for_topic(node, LaserScan, '/scan', 120)
                    and wait_for_topic(node, Odometry, '/odom', 120)):
                sys.exit('Gazebo did not come up in time — see logs/gazebo.log')

        stack = None
        for ctrl in controllers:
            if args.no_stack:
                print(f'--- controller: {ctrl} (external stack) ---')
            else:
                print(f'--- controller: {ctrl} ---')
                if ctrl in NAV2_CONTROLLERS:
                    cmd = (f'ros2 launch robot_navigation2 navigation2.launch.py '
                           f'controller:={ctrl} '
                           f'headless:={"false" if args.rviz else "true"}')
                else:
                    cmd = (f'ros2 launch robot_navigation2 tracker_bringup.launch.py '
                           f'tracker:={ctrl}')
                stack = spawn(cmd, log=log_dir / f'nav2_{ctrl}.log')
                # readiness
                from nav_msgs.msg import OccupancyGrid
                if not wait_for_topic(node, OccupancyGrid, '/map', 60):
                    sys.exit(f'no /map for {ctrl} — see logs/nav2_{ctrl}.log')
                from nav2_msgs.srv import ClearEntireCostmap
                from nav2_msgs.action import NavigateToPose
                wait_for_service(node, ClearEntireCostmap,
                                 '/global_costmap/clear_costmap', 60)
                if ctrl in NAV2_CONTROLLERS:
                    if not wait_for_action(node, NavigateToPose,
                                           '/navigate_to_pose', 60):
                        sys.exit(f'no /navigate_to_pose for {ctrl}')
                else:
                    if not wait_for_service(node, ClearEntireCostmap,
                                            '/local_costmap/clear_costmap', 5):
                        pass  # tracker mode has no local costmap

            for sc in scenarios:
                st, gl = sc['start'], sc['goal']
                for trial in range(args.trials):
                    print(f'  trial: {ctrl} / {sc["id"]} / #{trial}')
                    cmd = (f'ros2 run nav2_controller_benchmark evaluator --ros-args '
                           f'-p mode:={"nav2" if ctrl in NAV2_CONTROLLERS else "tracker"} '
                           f'-p controller:={ctrl} -p scenario_id:={sc["id"]} '
                           f'-p trial:={trial} '
                           f'-p start_x:={st["x"]} -p start_y:={st["y"]} -p start_yaw:={st["yaw"]} '
                           f'-p goal_x:={gl["x"]} -p goal_y:={gl["y"]} -p goal_yaw:={gl["yaw"]} '
                           f'-p output_csv:={csv_path} -p timeout:={args.timeout}')
                    ret = sh(cmd, log=log_dir / f'evaluator_{ctrl}_{sc["id"]}_{trial}.log')
                    if ret.returncode != 0:
                        print(f'    evaluator exited {ret.returncode} (see log)')

            if stack is not None:
                stop(stack, f'nav2_{ctrl}')
                stack = None

        if gazebo is not None:
            stop(gazebo, 'gazebo')

        print(f'\nCSV written: {csv_path}')
        if not args.no_analyze:
            import subprocess as sp
            sp.run([sys.executable,
                    str(REPO_ROOT / 'scripts' / 'analyze_results.py'),
                    '--csv', str(csv_path), '--out', str(out_dir / 'analysis')])
    finally:
        for proc, name in ((gazebo, 'gazebo'), (stack, 'nav2')):
            if proc is not None:
                stop(proc, name)
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#（注：内容由AI生成）
