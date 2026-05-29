# Opus Sim-to-Sim in rl_sar

目標：用 `/home/luis/unitree_rl_lab` 訓練出的 Opus policy，在 `/home/luis/rl_sar` 裡跑 ROS2/Gazebo sim-to-sim。

## 1. 已接入內容

IsaacLab policy 來源：

```text
/home/luis/unitree_rl_lab/logs/rsl_rl/unitree_opus_velocity/2026-05-22_18-16-55/exported/policy.pt
```

已複製到：

```text
/home/luis/rl_sar/policy/opus/legged_gym/policy.pt
/home/luis/rl_sar/policy/opus/robot_lab/policy.pt
```

目前 Opus FSM 進 RL 狀態時讀的是：

```text
opus/legged_gym/config.yaml
```

所以實際 sim-to-sim 會先使用：

```text
/home/luis/rl_sar/policy/opus/legged_gym/config.yaml
```

`robot_lab/config.yaml` 也已建立，內容和 `legged_gym` 對齊，方便之後把 FSM 改成 `robot_lab`。

## 2. 關鍵配置

Opus policy observation：

```text
ang_vel, gravity_vec, commands, dof_pos, dof_vel, actions
num_observations = 45
```

Opus action：

```text
num_of_dofs = 12
action_scale = 0.25 for all joints
default_dof_pos = [0.0, 0.8, -1.5] x 4 legs
joint_mapping = identity
```

注意：Go2 的 `robot_lab/config.yaml` 裡 hip action scale 是 `0.125`，Opus 不能照抄。Opus 在 IsaacLab 訓練時 `JointPositionActionCfg(scale=0.25)` 是全關節 0.25。

## 3. Opus description

已新增：

```text
/home/luis/rl_sar/src/rl_sar_zoo/opus_description
```

內容包括：

```text
urdf/opus_description.urdf
xacro/robot.xacro
xacro/gazebo.xacro
config/robot_control_ros2.yaml
meshes/
```

URDF mesh 路徑已改為：

```text
package://opus_description/meshes/...
```

Gazebo ROS2 使用：

```text
imu topic: /imu
ros2_control: gazebo_ros2_control/GazeboSystem
controller: robot_joint_controller/RobotJointControllerGroup
```

## 4. Build

```bash
cd /home/luis/rl_sar
source /opt/ros/humble/setup.bash
colcon build --merge-install --symlink-install --packages-select robot_msgs robot_joint_controller rl_sar opus_description
```

build 完後：

```bash
source /home/luis/rl_sar/install/setup.bash
```

## 5. 啟動 Gazebo

Terminal 1：

```bash
cd /home/luis/rl_sar
source /opt/ros/humble/setup.bash
source /home/luis/rl_sar/install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=opus
```

如果要 headless：

```bash
ros2 launch rl_sar gazebo.launch.py rname:=opus gui:=false
```

## 6. 啟動 rl_sar 控制

Terminal 2：

```bash
cd /home/luis/rl_sar
source /opt/ros/humble/setup.bash
source /home/luis/rl_sar/install/setup.bash
ros2 run rl_sar rl_sim
```

鍵盤控制：

```text
Num0 / A: get up
Num1: enter RL locomotion
Num9 / B: get down
P: passive
R: reset simulation
Enter: pause / unpause simulation
```

## 7. cmd_vel 測試

如果需要用 `/cmd_vel`：

```bash
cd /home/luis/rl_sar
source /opt/ros/humble/setup.bash
source /home/luis/rl_sar/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

注意：rl_sar 裡需要切到 navigation mode 才會用 `/cmd_vel`，否則使用鍵盤/手柄控制量。

## 8. 已驗證

已完成：

```text
opus_description build OK
robot_msgs / robot_joint_controller / rl_sar build OK
xacro can expand
rl_sim dynamic libraries resolve after sourcing install/setup.bash
ros2 launch rl_sar gazebo.launch.py --show-args OK
```

## 9. 風險點

第一輪 sim-to-sim 主要看：

```text
1. Gazebo 裡 Opus 初始高度和姿態是否正常
2. robot_joint_controller 是否收到 12 個 joint
3. /imu 是否正常發布
4. Num0 get up 是否能站穩
5. Num1 進 RL 後是否能維持姿態
6. command 方向和實際移動方向是否一致
```

如果進 RL 後抽搐或方向不對，優先查：

```text
joint_mapping
joint axis / sign
default_dof_pos
action_scale
IMU frame / quaternion convention
Gazebo self collision / contact
```
