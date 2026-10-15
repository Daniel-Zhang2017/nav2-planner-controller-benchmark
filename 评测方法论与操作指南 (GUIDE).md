# Nav2 本地控制器对比评估指南（Controller Benchmark Guide）

本指南说明如何在 `robot009` 仿真环境（Gazebo + Nav2 Humble）中，对 **DWB / MPPI / RPP / PID-Safe / LQR-Safe** 五个本地控制器做**自动化、可复现、公平**的对比，以及如何读懂结果。

> 配套代码在本仓库 `src/`、`scripts/` 中，与 `Daniel-Zhang2017/ros2-robotics-projects` 的 `project0/robot009` 共用同一套机器人模型（差分驱动、激光雷达、IMU、相机）和同一张地图（`room`）。

---

## 1. 五个控制器在评测中的身份

| 控制器 | 运行方式 | Nav2 插件 / 节点 | 局部避障能力 | 速度上限（评测统一） |
|---|---|---|---|---|
| DWB | Nav2 原生 | `dwb_core::DWBLocalPlanner`（controller_server 插件） | 轨迹评分含障碍代价 | 0.26 m/s, 0.8 rad/s |
| MPPI | Nav2 原生 | `nav2_mppi_controller::MPPIController` | 采样 + ObstaclesCritic | 0.26 m/s, 0.8 rad/s |
| RPP | Nav2 原生 | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | 碰撞检测 + 曲率/靠近障碍减速 | 0.26 m/s, 0.8 rad/s |
| PID-Safe | 外部 tracker 节点 | `pid_safe_controller`（自研，Python） | **仅激光前向锥急停**，无本地避障 | 0.26 m/s, 0.8 rad/s |
| LQR-Safe | 外部 tracker 节点 | `lqr_safe_controller`（自研，Python） | **仅激光前向锥急停**，无本地避障 | 0.26 m/s, 0.8 rad/s |

**重要说明（公平性边界）**：PID-Safe / LQR-Safe 是绕过 controller_server 的路径跟踪器（这是你 README 里已经定义的设计），它们没有 Nav2 的本地代价地图避障，也没有行为恢复（spin/backup）。所以评测结论应解读为：

- DWB / MPPI / RPP 之间 → "同为 Nav2 原生控制器，谁的局部规划更好"；
- PID-Safe / LQR-Safe 之间 → "同为纯跟踪器，PID vs LQR 控制律差异"；
- 两组之间 → "带避障的规划控制器 vs 纯跟踪 + 急停"的能力边界，不是同能力等级的胜负。

---

## 2. 评测方法：为什么这样测

### 2.1 公平性控制（消除混杂变量）

1. **同一张地图、同一组 start/goal**：场景由 `scripts/generate_scenarios.py` 从 `room.pgm` 自动生成并验证（起点/终点是自由格；`detour` 场景的直线必经墙，`straight` 场景直线畅通）。
2. **同一速度上限**：所有控制器的 `vx_max 0.26 / wz_max 0.8 / ax 2.5 / az 2.0` 与你的 MPPI 原配置一致（见 `scripts/make_controller_params.py` 内注释）。
3. **同一全局规划器**：全部使用 `NavfnPlanner`（`use_astar: false`），排除全局规划器差异。
4. **同一仿真复位流程**：每次 trial 前 evaluator 都会（a）`/set_entity_state` 把 Gazebo 里的机器人重置到起点、（b）向 AMCL 发布 `/initialpose`、（c）清空 costmap —— 保证每次 trial 的起始状态一致。
5. **同一输出链路**：tracker 输出经 `cmd_vel_nav → velocity_smoother → /cmd_vel`，与 Nav2 原生控制器走完全相同的加速度平滑（Humble 官方 remap 方案）。
6. **多 trial 统计**：每个 场景×控制器 至少跑 3 次（建议 5 次），报告 mean ± std，不拿单次结果下结论。

### 2.2 评测流程

```
Gazebo(世界+机器人) ──> Nav2 栈(按控制器切换参数) ──> evaluator 逐 trial:
   重置位姿 → 设初始位姿 → 清代价地图 → 发目标
   → 记录 odom / scan / cmd_vel / 全局路径
   → 成功 | 卡死 | 超时 | 失败
   → 计算指标 → 写入 run.csv
run.csv ──> analyze_results.py ──> summary.md + summary.csv + plots/*.png
```

### 2.3 指标定义（口径与 metrics.py 一致）

| 指标 | 定义 | 好坏的直觉 |
|---|---|---|
| `success` | trial 是否在超时内到达（Nav2 模式取 BT 结果；tracker 模式取里程计距离+航向双容差） | 越高越好 |
| `completion_time_s` | 从发目标到完成（模拟时钟） | 越低越好 |
| `traveled_dist_m` | 里程计轨迹弧长 | 越接近最短绕行越好 |
| `cte_mean/cte_max_m` | 到全局路径**最近线段**的垂距，全程均值/最大值 | 越低说明贴线越准 |
| `cte_*_initial_m` | 相对**首条规划路径**（复规划前）的 CTE | 反映初始跟踪质量 |
| `min_clearance_forward_m` | 全程中激光前向锥(±60°)的最小测距 | 越大越安全（离墙越远） |
| `jerk_lin/ang` | cmd_vel 的一阶差分加速度再差分的二阶导，均值/峰值 | 越小越平顺 |
| `lin_oscillations` | cmd_vel 线速度符号变化次数（死区 0.01） | 越小越稳 |
| `avg_lin_vel / avg_ang_vel` | 指令速度绝对值均值 | 侧面反映激进/保守 |
| `collision` | 机器人中心进入地图障碍距离 < 机器人半径的时间占比（由 /map 距离变换判定） | 越接近 0 越好 |

> 注意：`min_clearance_all_m`（全向最小测距）通常贴着墙走也会很小，参考价值低于前向锥版本。

---

## 3. 目录结构

```
nav2-planner-controller-benchmark/
├── README.md
├── requirements.txt
├── scripts/
│   ├── generate_scenarios.py      # 从地图自动生成并验证场景
│   ├── make_controller_params.py  # 从基础参数生成 mppi/dwb/rpp 三份配置
│   ├── run_benchmark.py           # 全自动跑批：起仿真→切控制器→逐 trial 评测
│   └── analyze_results.py         # 汇总表格 + 箱线图 + Markdown 报告
├── src/
│   ├── robot_description/         # 机器人模型 + custom_room.world（与 robot009 相同）
│   ├── robot_navigation2/
│   │   ├── config/
│   │   │   ├── nav2_params.yaml           # 默认（MPPI，与主仓库一致）
│   │   │   ├── controllers/
│   │   │   │   ├── _base_nav2_params.yaml # 公共部分唯一来源
│   │   │   │   ├── mppi.yaml / dwb.yaml / rpp.yaml
│   │   │   │   ├── tracker_base.yaml      # tracker 模式参数（无 controller_server）
│   │   │   │   └── benchmark/scenarios.yaml  # 自动生成的场景
│   │   │   └── maps/room.*
│   │   └── launch/
│   │       ├── navigation2.launch.py      # 支持 controller:= / headless:=
│   │       └── tracker_bringup.launch.py  # tracker 模式 bringup
│   └── nav2_controller_benchmark/         # 新 Python 包
│       ├── nav2_controller_benchmark/
│       │   ├── metrics.py                 # 纯函数指标（无 ROS 依赖，可单测）
│       │   ├── tracker_base.py            # tracker 公共基类
│       │   ├── pid_safe_controller.py
│       │   ├── lqr_safe_controller.py
│       │   └── evaluator.py               # 单次 trial 全流程 + CSV 输出
│       └── setup.py / package.xml
└── results/                               # run.csv、analysis/（自动生成）
```

---

## 4. 快速开始

### 4.1 依赖（Ubuntu 22.04 + ROS2 Humble）

```bash
sudo apt update
sudo apt install ros-humble-nav2-mppi-controller \
                 ros-humble-nav2-regulated-pure-pursuit-controller \
                 ros-humble-nav2-dwb-controller   # 若无则 apt 搜索 nav2-dwb
pip3 install -r requirements.txt                # numpy/pandas/matplotlib/scipy/pyyaml
```

> DWB 一般随 nav2 默认安装；如缺失：`apt search nav2 | grep dwb`。

### 4.2 编译

```bash
cd nav2-planner-controller-benchmark
colcon build
source install/setup.bash
```

### 4.3 一键跑全量对比（5 控制器 × 5 场景 × 3 trial，headless）

```bash
python3 scripts/run_benchmark.py \
    --controllers mppi,dwb,rpp,pid_safe,lqr_safe \
    --trials 3 \
    --analyze
```

跑完后 `results/run_<时间戳>/` 下有：

- `run.csv` —— 每个 trial 一行（完整明细）；
- `analysis/summary.md` —— 可读对比报告；
- `analysis/summary.csv` / `by_scenario.csv` —— 统计表；
- `analysis/plots/*.png` —— 成功率、CTE、到达时间、最小前向间隙、jerk、震荡、Pareto 图。

### 4.4 手动单控制器调试（不开跑批，便于看 RViz）

```bash
# 终端 1：仿真
ros2 launch robot_description gazebo_robot.launch.py
# 终端 2：Nav2（DWB 为例；可换 mppi/rpp）
ros2 launch robot_navigation2 navigation2.launch.py controller:=dwb
# 终端 3：单次评测
ros2 run nav2_controller_benchmark evaluator --ros-args \
    -p mode:=nav2 -p controller:=dwb -p scenario_id:=g1_detour -p trial:=0 \
    -p start_x:=0.0 -p start_y:=0.0 -p start_yaw:=0.0 \
    -p goal_x:=-8.0 -p goal_y:=-5.5 -p goal_yaw:=0.0 \
    -p output_csv:=/tmp/run.csv -p timeout:=120.0
```

tracker 模式手动调试：

```bash
# 终端 2：tracker bringup（pid_safe / lqr_safe）
ros2 launch robot_navigation2 tracker_bringup.launch.py tracker:=lqr_safe
# 终端 3：单次评测（mode:=tracker）
ros2 run nav2_controller_benchmark evaluator --ros-args \
    -p mode:=tracker -p controller:=lqr_safe ...（同上参数）
```

### 4.5 自定义场景与参数

```bash
# 重新生成场景（例如要求更多/更远的点）
python3 scripts/generate_scenarios.py --map src/robot_navigation2/maps/room.yaml \
    --start 0 0 0 --detour 3 --straight 2 \
    --out src/robot_navigation2/config/benchmark/scenarios.yaml

# 调完公共参数后重新生成三份 controller 配置
python3 scripts/make_controller_params.py
```

---

## 5. 调参与扩展

### 5.1 PID-Safe 参数（`pid_safe_controller`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `kp_heading` | 2.0 | 航向误差比例增益 |
| `kd_heading` | 0.2 | 航向误差微分增益（抑制振荡） |
| `kp_cte` | 0.8 | 横向偏差反馈增益（贴线） |
| `lookahead` | 0.5 m | 前视距离 |
| `speed_gain` | 0.6 | v = clamp(speed_gain × 到目标距离) |
| `brake_dist` | 0.25 m | 前向锥急停阈值 |

调参顺序：先定 `lookahead`（车小快跑用 0.4–0.6），再加大 `kp_heading` 直到贴线且不抖，`kd_heading` 压振荡，`kp_cte` 修正侧偏。

### 5.2 LQR-Safe 参数（`lqr_safe_controller`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `q_cte` | 10.0 | 横向偏差权重 |
| `q_heading` | 1.0 | 航向误差权重 |
| `r_omega` | 0.5 | 控制量（角速度）惩罚 |
| `v0` | 0.26 | 线性化速度（模型矩阵） |

离散代数 Riccati 方程在启动时用 scipy 求解一次，得到增益 `K`，`w = -K·[cte, heading_err]`（随后按 `wz_max` 和加速度斜坡限幅）。想要更"柔和"：增大 `r_omega` / 减小 `q_cte`；想要更"激进贴线"：反之。

### 5.3 加入新控制器

- **Nav2 原生插件**（如 TEB）：在 `scripts/make_controller_params.py` 的 `BLOCKS` 里加一段 `FollowPath` 配置 → 生成新 yaml → 在 `run_benchmark.py` 的 `NAV2_CONTROLLERS` 里加名字即可。
- **外部 tracker**：继承 `tracker_base.py`，实现 `compute_control()`，在 `setup.py` 注册 entry point，并在 `tracker_bringup.launch.py` 加对应节点。

---

## 6. 常见问题（Troubleshooting）

| 现象 | 排查 |
|---|---|
| `controller_server` 启动即崩 | 检查 `config/controllers/<name>.yaml` 的 `plugin` 拼写；`ros2 launch ... controller:=dwb` 后看 `ros2 log show controller_server` |
| RPP 报参数警告 | Humble 版参数名以 `docs.ros.org/en/humble/p/nav2_regulated_pure_pursuit_controller/` 为准（如 `max_allowed_time_to_collision_up_to_carrot`、`use_regulated_linear_velocity_scaling`）；未知参数仅警告不致命 |
| tracker 模式 evaluator 卡在等 /map | 确认 `tracker_bringup.launch.py` 生命周期管理器已 autostart（map_server/amcl/planner_server 都要 active） |
| MPPI 卡死/不前进 | 经典 MPPI 调参问题，参考主仓库 README 的 Project 0.9 章节（critic 权重、ObstaclesCritic 依赖 local costmap） |
| 仿真重复 trial 后 AMCL 漂移 | 拉大 `reset_robot()` 与 `/initialpose` 之间的等待；或 `--no-restart-sim` 改为每次重启 Gazebo（更慢但更干净） |
| 结果方差很大 | 增加 `--trials`；确认没有后台进程占 CPU 导致仿真变慢 |
| 想记录更多话题（如恢复行为次数） | 在 `evaluator.py` 加订阅，在 `CSV_COLUMNS` 加列，在 `compute_row()` 里算 |

---

## 7. 预期结果与解读建议

- **MPPI 在窄缝/绕障场景成功率通常最高**（采样式多轨迹），代价是计算量大、且参数敏感；
- **RPP 在畅通场景到达时间通常最短**（纯跟踪 + 曲率减速），但在必须大绕行的窄空间可能贴墙过近甚至失败；
- **DWB 居中**，轨迹评分直观，但震荡（oscillations）常高于 MPPI；
- **PID-Safe / LQR-Safe 贴线精度（CTE）通常最好**（直接跟踪全局路径），但一旦遇到规划路径没覆盖的新障碍只有急停没有绕行，成功率受规划质量限制。

看结论时请同时看 **成功率 × 时间 × CTE × 最小间隙** 四个维度（Pareto 图：左下角最优），不要只看单一指标。

---

## 8. 数据来源与引用

- RPP Humble 参数名与行为描述：Nav2 官方文档
  http://docs.ros.org/en/humble/p/nav2_regulated_pure_pursuit_controller/
- Nav2 Controller Server 与插件配置：
  https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/controller_server/
- Humble bringup topic remap（cmd_vel_nav → velocity_smoother → cmd_vel）：
  https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/navigation_launch.py
- MPPI 配置说明：https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/
- 主仓库 robot009 MPPI 踩坑记录：`Daniel-Zhang2017/ros2-robotics-projects` README Project 0.9
