# Nav2 Planner Controller Benchmark

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![Nav2](https://img.shields.io/badge/Navigation2-Supported-orange.svg)](https://navigation.ros.org/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-yellow.svg)](https://www.python.org/)

**Nav2 Local Controller Automated Comparison Benchmark** — Gazebo simulation environment, identical map, identical scenarios, and identical velocity limits, automatically comparing 5 local controllers and generating a readable report.

## Controllers Under Test

| Controller | Execution Mode | Plugin / Node |
|---|---|---|
| **DWB** | Nav2 Native | `dwb_core::DWBLocalPlanner` |
| **MPPI** | Nav2 Native | `nav2_mppi_controller::MPPIController` |
| **RPP** | Nav2 Native | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |
| **PID-Safe** | External Tracker (Custom) | `pid_safe_controller` (PID tracking + laser emergency stop) |
| **LQR-Safe** | External Tracker (Custom) | `lqr_safe_controller` (Discrete LQR state feedback + laser emergency stop) |

> **Note:** PID-Safe and LQR-Safe bypass `controller_server`, performing global path tracking and laser forward-cone emergency stops only (without local costmap obstacle avoidance). This is a well-defined capability boundary by design. See `docs/CONTROLLER_BENCHMARK_GUIDE.md` for details.

---

## Automated Evaluation

Each trial workflow follows this sequence: 
**Reset Robot ➔ Set Initial Pose ➔ Clear Costmaps ➔ Send Goal ➔ Record Data (`odom`/`scan`/`cmd_vel`/`plan`) ➔ Calculate Metrics ➔ Write CSV**

Core metrics include: Success rate, travel time, travel distance, cross-track error (CTE), minimum forward clearance, jerk, oscillation count, and collision ratio.

```text
Gazebo ─► Nav2 (Switch Params per Controller) ─► Evaluator (Per Trial) ─► run.csv ─► Analyze ─► Summary + Plots
Quick Start (ROS2 Humble)
1) Dependencies & Build
Bash


# Install required controller packages
sudo apt install ros-humble-nav2-mppi-controller \
                 ros-humble-nav2-regulated-pure-pursuit-controller

# Install Python dependencies
pip3 install -r requirements.txt

# Build workspace
colcon build && source install/setup.bash
2) Run Full Benchmark
Execute a full batch comparison (5 controllers × 5 scenarios × 3 trials, headless mode):

Bash


python3 scripts/run_benchmark.py --controllers mppi,dwb,rpp,pid_safe,lqr_safe --trials 3
Reports will be generated in results/run_*/analysis/ (containing summary.md and plots/*.png).

3) Manual Debugging (Single Controller)
Bash


# Terminal 1: Launch Gazebo simulation
ros2 launch robot_description gazebo_robot.launch.py

# Terminal 2: Launch Navigation2 with a specific controller (e.g., DWB)
ros2 launch robot_navigation2 navigation2.launch.py controller:=dwb

# Terminal 3: Run evaluator for a specific trial
ros2 run nav2_controller_benchmark evaluator --ros-args \
    -p mode:=nav2 -p controller:=dwb -p scenario_id:=g1_detour -p trial:=0 \
    -p start_x:=0.0 -p start_y:=0.0 -p start_yaw:=0.0 \
    -p goal_x:=-8.0 -p goal_y:=-5.5 -p goal_yaw:=0.0 -p output_csv:=/tmp/run.csv
For tracker mode: Use ros2 launch robot_navigation2 tracker_bringup.launch.py tracker:=lqr_safe and set evaluator parameter -p mode:=tracker.

Directory Structure
scripts/ — Scenario generation, parameter generation, batch running, and analysis tools (see docs/CONTROLLER_BENCHMARK_GUIDE.md).

src/robot_description/ — Robot URDF/Xacro model + custom_room.world.

src/robot_navigation2/ — Nav2 parameter profiles (one YAML file per controller) and launch files.

src/nav2_controller_benchmark/ — Evaluator node, custom PID/LQR trackers, and metric calculators.

results/ — Run CSV logs and analysis artifacts (auto-generated).

Python Dependencies
Plaintext


numpy
pandas
matplotlib
scipy
pyyaml