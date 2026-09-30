# Kryptonitex Smart Warehouse

**Edge-AI Based Distributed Fleet Coordination for Autonomous Mobile Robots (AMRs) in Smart Warehouses**

ROS 2 Jazzy + Gazebo Harmonic warehouse simulation for the **Kryptonitex** team.

## What is included

- `worlds/final_smart_warehouse_48x48x3.sdf` — the 48 m × 48 m warehouse world supplied for this project.
- `src/forklift_robot/` — a Gazebo Harmonic / `gz_ros2_control` forklift model adapted from the dimensions and structure of the public forklift reference listed in `third_party/ATTRIBUTIONS.md`.
- `src/task_msgs/` — task, robot-state and assignment messages.
- `src/task_allocator/` — decentralized-style local bid allocator based on distance, battery, workload and task priority.
- `src/astar_planner/` — grid A* planner that can extract rack collision geometry from the warehouse SDF and publish a `nav_msgs/Path`.
- `launch/warehouse_demo.launch.py` — starts the warehouse and forklift stack.
- `config/warehouse.yaml` — map, robot and planner parameters.

## Architecture

```text
Task / Robot State
        |
        v
  Local Task Allocator
        |
        v
   Task Assignment
        |
        v
      A* Planner <---- warehouse SDF rack geometry
        |
        v
     nav_msgs/Path
        |
        v
 Forklift / AMR Controller
        |
        v
   Gazebo Harmonic
```

The repository intentionally keeps the A* planner and allocator as original Kryptonitex integration code. The external repositories are referenced and credited rather than copied wholesale because the upstream repositories do not declare a reusable license in their package metadata.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- `ros-jazzy-ros-gz-sim`
- `ros-jazzy-ros-gz-bridge`
- `ros-jazzy-gz-ros2-control`
- `ros-jazzy-ros2-control`
- `ros-jazzy-ros2-controllers`
- `ros-jazzy-nav-msgs`
- `ros-jazzy-geometry-msgs`
- `ros-jazzy-xacro`

The ROS/Gazebo integration combination is supported by `ros_gz`, and `gz_ros2_control` provides the Gazebo-Sim ros2_control integration used by this project. See the official documentation linked in `third_party/ATTRIBUTIONS.md`.

## Build

```bash
cd ~/kryptonitex-smart-warehouse
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run the warehouse

```bash
source /opt/ros/jazzy/setup.bash
source ~/kryptonitex-smart-warehouse/install/setup.bash
ros2 launch forklift_robot warehouse_demo.launch.py
```

The launch file starts Gazebo Harmonic, publishes the forklift robot description, spawns the forklift, and loads the diff-drive and joint-state controllers.

## Test the task allocator + A*

Terminal 1 — warehouse/forklift:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kryptonitex-smart-warehouse/install/setup.bash
ros2 launch forklift_robot warehouse_demo.launch.py
```

Terminal 2 — allocator:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kryptonitex-smart-warehouse/install/setup.bash
ros2 run task_allocator task_allocator_node
```

Terminal 3 — A* planner:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kryptonitex-smart-warehouse/install/setup.bash
ros2 run astar_planner astar_planner_node --ros-args \
  -p sdf_path:=$(ros2 pkg prefix forklift_robot)/share/forklift_robot/worlds/final_smart_warehouse_48x48x3.sdf
```

Terminal 4 — send a demo robot state and task:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kryptonitex-smart-warehouse/install/setup.bash
python3 scripts/send_demo_task.py
```

Terminal 5 — inspect the generated path:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kryptonitex-smart-warehouse/install/setup.bash
ros2 topic echo /astar/path
```

The planner reads rack collision boxes from the warehouse SDF, inflates them by the configured robot radius, and runs grid A* from the latest robot position to the assigned task position. Example message definitions are documented in `docs/interfaces.md`.

## Important note about the external forklift repository

The reference repository was developed for ROS 2 Humble / Gazebo Classic 11 according to its README. This repository therefore does **not** blindly copy its Classic Gazebo plugins into the Jazzy/Harmonic project. Instead, the forklift geometry is recreated as a Harmonic-compatible model and the control interface is implemented with `gz_ros2_control`.

## License

Original Kryptonitex files are released under MIT. See `LICENSE`.

The warehouse SDF is project-owned input supplied for this repository. External reference repositories remain the property of their respective authors and are listed in `third_party/ATTRIBUTIONS.md`.
