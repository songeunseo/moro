# moro

## Run Order

### 1. Terminal 1: start the simulation

```bash
cd /opt/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch moro_maze simulation_launch.py
```

If the package has not been built yet:

```bash
cd /opt/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select moro_maze
source install/setup.bash
```

### 2. Terminal 2: start the global planner

```bash
cd /opt/ros2_ws
source install/setup.bash
ros2 run moro_maze global_planner
```

### 3. Set the initial pose

In RViz, use `2D Pose Estimate` to set the robot's current position on the map.

### 4. Terminal 3: start the local controller

```bash
cd /opt/ros2_ws
source install/setup.bash
ros2 run moro_maze local_control
```
