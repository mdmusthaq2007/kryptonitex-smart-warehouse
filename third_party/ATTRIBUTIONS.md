# Third-party references and attribution

## Forklift reference

Reference repository:
https://github.com/cangozpi/ROS2-Forklift-Simulation

The upstream README describes a ROS 2 forklift model with URDF, controllers and sensors. It also states that the reference environment was developed with ROS 2 Humble and Gazebo 11. The upstream `package.xml` currently contains a `TODO: License declaration`, so this repository does not copy the upstream package wholesale. Instead, `src/forklift_robot/` is a fresh Gazebo Harmonic-compatible implementation using the publicly visible forklift dimensions/structure as engineering reference.

## Task allocation reference

Reference repository:
https://github.com/Chaaru-Mukesh/sih-amr-fleet

The reference repository contains a small ROS 2 distributed multi-AGV task-allocation example with `task_msgs` and a `task_allocator` package. The Kryptonitex implementation uses the same high-level interface idea (robot state → task → assignment) but the code in this repository is newly written for this project.

## ROS/Gazebo integration

- ROS-Gazebo integration: https://github.com/gazebosim/ros_gz
- Gazebo ros2_control integration: https://github.com/ros-controls/gz_ros2_control

Official documentation:
https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html
