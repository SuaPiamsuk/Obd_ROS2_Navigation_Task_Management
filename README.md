# Obd_ROS2_Navigation_Task_Management

## System requirement
1. Ubuntu 20.04
2. Ros2 Foxy

## Quick start
1. spawn robot(Turtlebot3) in 35 x 35 m^2

```
ros2 launch obd_gazebo turtlebot3_world.launch.py
```

2. slam

```
ros2 launch obd_navigation slam.launch.py
```

3. navigation2

```
ros2 launch obd_navigation navigation2.launch.py
```
