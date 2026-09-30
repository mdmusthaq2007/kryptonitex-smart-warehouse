# ROS interfaces

## `/robot_states`

Type: `task_msgs/msg/RobotState`

```text
string robot_id
float64 x
float64 y
float64 battery
int32 workload
```

## `/task`

Type: `task_msgs/msg/Task`

```text
float64 x
float64 y
int32 priority
```

## `/task_assignment`

Type: `task_msgs/msg/TaskAssignment`

```text
string robot_id
float64 x
float64 y
```

## `/astar/path`

Type: `nav_msgs/msg/Path`

The path starts at the latest position received for the assigned robot and ends at the assigned task coordinates.
