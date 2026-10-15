# nav2-planner-controller-benchmark

**Nav2 本地控制器自动化对比评测** —— Gazebo 仿真环境，同一张地图、同一组场景、同一速度上限，自动对比 5 个本地控制器并输出可读报告。

Controllers under test:

| controller | 运行方式 | 插件 / 节点 |
|---|---|---|
| DWB | Nav2 原生 | `dwb_core::DWBLocalPlanner` |
| MPPI | Nav2 原生 | `nav2_mppi_controller::MPPIController` |
| RPP | Nav2 原生 | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |
| PID-Safe | 外部 tracker（自研） | `pid_safe_controller`（PID 跟踪 + 激光急停） |
| LQR-Safe | 外部 tracker（自研） | `lqr_safe_controller`（离散 LQR 状态反馈 + 激光急停） |

> PID-Safe / LQR-Safe 绕过 controller_server，只做全局路径跟踪 + 激光前向锥急停（无本地代价地图避障），这是设计上明确的能力边界。详见 `docs/CONTROLLER_BENCHMARK_GUIDE.md`。

## 自动化评估

每次 trial：复位机器人 → 设初始位姿 → 清代价地图 → 发目标 → 记录 odometry/scan/cmd_vel/全局路径 → 计算指标 → 写 CSV。指标包括：成功率、到达时间、行驶距离、横向误差（CTE）、最小前向间隙、jerk、震荡次数、碰撞占比。

```
Gazebo ─ Nav2(按控制器切参数) ─ evaluator(逐 trial) ─ run.csv ─ analyze ─ summary + plots
```

## 快速开始（ROS2 Humble）

```bash
# 1) 依赖
sudo apt install ros-humble-nav2-mppi-controller \
                 ros-humble-nav2-regulated-pure-pursuit-controller
pip3 install -r requirements.txt

# 2) 编译
colcon build && source install/setup.bash

# 3) 一键全量对比（5 控制器 × 5 场景 × 3 trial，headless）
python3 scripts/run_benchmark.py --controllers mppi,dwb,rpp,pid_safe,lqr_safe --trials 3

# 4) 报告在 results/run_*/analysis/（summary.md + plots/*.png）
```

单控制器手动调试：

```bash
ros2 launch robot_description gazebo_robot.launch.py                 # 终端 1
ros2 launch robot_navigation2 navigation2.launch.py controller:=dwb  # 终端 2
ros2 run nav2_controller_benchmark evaluator --ros-args \
    -p mode:=nav2 -p controller:=dwb -p scenario_id:=g1_detour -p trial:=0 \
    -p start_x:=0.0 -p start_y:=0.0 -p start_yaw:=0.0 \
    -p goal_x:=-8.0 -p goal_y:=-5.5 -p goal_yaw:=0.0 -p output_csv:=/tmp/run.csv
```

tracker 模式：`ros2 launch robot_navigation2 tracker_bringup.launch.py tracker:=lqr_safe`，evaluator 用 `-p mode:=tracker`。

## 目录

- `scripts/` —— 场景生成、参数生成、跑批、分析（详见 `docs/CONTROLLER_BENCHMARK_GUIDE.md`）
- `src/robot_description/` —— 机器人模型 + `custom_room.world`（与 robot009 相同）
- `src/robot_navigation2/` —— Nav2 参数（每控制器一份 yaml）、launch
- `src/nav2_controller_benchmark/` —— evaluator + PID/LQR tracker + metrics
- `results/` —— run.csv 与分析产物（自动生成）

## Python 依赖

```bash
pip3 install -r requirements.txt
# numpy, pandas, matplotlib, scipy, pyyaml
```
