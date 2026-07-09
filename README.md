# moro

2026-1 Mobile Robotics Final Project

## Package

This repository contains the `moro_maze` ROS 2 package. The project launches a
TurtleBot3 simulation, loads the maze map, plans a path through the maze, and
drives the robot with a local controller.

## Build

From the workspace root:

```bash
colcon build --packages-select moro_maze
source install/setup.bash
```

If you open a new terminal, source the workspace again before running ROS 2
commands:

```bash
source install/setup.bash
```

## Start With One Command

The main launch file starts the simulation, robot spawn, RViz, transform setup,
global planner, and local controller:

```bash
ros2 launch moro_maze simulation_launch.py
```

This is the recommended command for quickly starting the complete project.

The default robot start position is `(2.0, 1.0)`. To choose another start
position, pass `x_pose` and `y_pose`:

```bash
ros2 launch moro_maze simulation_launch.py x_pose:=1.0 y_pose:=3.0
```

In RViz, the `2D Pose Estimate` tool can be used to send a new initial pose on
`/initialpose`. The global planner listens to this topic and replans the path
from the selected pose.

Note: `2D Pose Estimate` changes the planner's start estimate; it does not
respawn or teleport the Gazebo robot. To change the physical spawn position of
the simulated robot, use `x_pose` and `y_pose` when launching.

## Start Nodes Individually

For demonstrations or debugging, the nodes can also be started in separate
terminals so their log output is easier to distinguish.

Terminal 1:

```bash
ros2 launch moro_maze simulation_only_launch.py
```

You can pass the same start position arguments here:

```bash
ros2 launch moro_maze simulation_only_launch.py x_pose:=1.0 y_pose:=3.0
```

Terminal 2:

```bash
ros2 run moro_maze global_planner
```

Terminal 3:

```bash
ros2 run moro_maze local_control
```

This keeps the planner and controller logs in their own terminals.

## Notes

- The robot starts at `(2.0, 1.0)`.
- The configured goal is `(4.0, 4.0)`.
- `moro_maze/maps/map.yaml` is used by the global planner.
- `moro_maze/rviz/config.rviz` is loaded for visualization.
