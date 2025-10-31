# Overview
This is a program used to bridge MSI AMR to a ROS2 node.

- Feature




# How to build

1. Obtaining Source Code

```
$ cd <ROS2_workspace>/src
```

2. Installing Related Packages

```
```

3. Build

```
colcon build --merge-install --packages-select player_bridge --event-handlers console_cohesion+
```

#### run player_bridge
source install/share/player_bridge/environment/99_player_lib.sh
ros2 run player_bridge player_bridge --ros-args -p player_host:=192.168.0.1


