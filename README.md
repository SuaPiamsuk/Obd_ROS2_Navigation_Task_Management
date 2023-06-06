# Obd_ROS2_Navigation_Task_Management

## System requirement
1. Ubuntu 20.04
2. ROS2 Foxy

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

5. send goal from file

```
ros2 run obd_navigation cmd_nav2.py
```

4. service call

```
ros2 service call /send_goal_nav2 obd_interfaces/srv/Goal "{x: 1.0,y: 0.0,theta: 3.14}"
```

wait at waypoint?
https://navigation.ros.org/configuration/packages/configuring-waypoint-follower.html
