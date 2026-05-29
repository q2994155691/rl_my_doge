# A1 Gazebo Sim Commands

## Rebuild after code changes

```bash
cd ~/rl_sar
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
colcon build --merge-install --packages-select rl_sar
```

## ROS2 Gazebo

Terminal 1:

```bash
cd ~/rl_sar
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=a1
```

Terminal 2:

```bash
cd ~/rl_sar
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 run rl_sar rl_sim
```

## ROS1 Gazebo

Terminal 1:

```bash
cd ~/rl_sar
source devel/setup.bash
roslaunch rl_sar gazebo.launch rname:=a1
```

Terminal 2:

```bash
cd ~/rl_sar
source devel/setup.bash
rosrun rl_sar rl_sim
```

## Keyboard Controls

```text
0      GetUp
3      Trot
P      Passive
9      GetDown
R      Reset Gazebo
Enter  Pause/resume Gazebo
W/S    Increase/decrease x command
A/D    Increase/decrease y command
Q/E    Increase/decrease yaw command
Space  Clear x/y/yaw commands
```

## Gamepad Controls

```text
Left stick Y  x command
Left stick X  y command
Right stick X yaw command
```
