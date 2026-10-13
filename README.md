# nav2-planner-controller-benchmark

Benchmark for Nav2 global planners \& local controllers. Compare DWB, MPPI, RPP, PID, LQR‑Safe with automatic evaluation and quantitative metrics.

# nav2‑planner‑controller‑benchmark

This repository provides a complete simulation framework to benchmark and compare different global planners and local motion controllers for ROS 2 Nav2.

Controllers under test:
- DWB (Dynamic Window Approach)
- MPPI (Model Predictive Path Integral)
- RPP (Regulated Pure Pursuit)
- PID‑Safe (PID path‑tracker with laser emergency brake)
- LQR‑Safe (LQR state‑feedback tracker with laser emergency brake)

Global planners supported:
- SmacPlanner2D, SmacPlannerHybrid, NavFn Planner

### Key features
✅ Gazebo‑based consistent simulation testbed
✅ Automatic experiment evaluator node: record odometry, scan, cross‑track error, minimum obstacle distance, jerk into CSV
✅ Auto‑stop recording once navigation goal is reached
✅ Post‑processing Python scripts: generate summary table and comparison plots
✅ Fair comparison under identical map, start‑goal pose and velocity constraints

### Metrics
- Average / maximum cross‑track error (CTE)
- Minimum obstacle distance
- Linear & angular jerk
- Mission completion time

> Note: PID‑Safe and LQR‑Safe are external tracker nodes bypassing Nav2 controller_server, with only laser‑based emergency braking (no native local obstacle avoidance).


\#### Python dependencies

```bash

pip3 install -r requirements.txt

```

