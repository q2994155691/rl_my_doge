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
opus/robot_lab/config.yaml
```

所以實際 sim-to-sim 會先使用：

```text
/home/luis/rl_sar/policy/opus/robot_lab/config.yaml
```

`legged_gym/config.yaml` 保留作對照用。

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
default_dof_pos = [
  0.10, -0.10, 0.10, -0.10,
  0.80,  0.80, 1.00,  1.00,
 -1.50, -1.50, -1.50, -1.50
]
joint_mapping = [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]
commands_scale = [1.0, 1.0, 1.0]
ang_vel_scale = 0.2
rl_sim_kp = 10.0 for all joints
rl_sim_kd = 0.05 for all joints
```

注意：Go2 的 `robot_lab/config.yaml` 裡 hip action scale 是 `0.125`，Opus 不能照抄。Opus 在 IsaacLab 訓練時 `JointPositionActionCfg(scale=0.25)` 是全關節 0.25。

`joint_mapping` 是 policy joint order 到 rl_sar raw joint order 的映射；raw joint order 來自 `policy/opus/base.yaml`。

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

## 9. MuJoCo rl_sim_mujoco

`rl_sim_mujoco` 是非 ROS 的 MuJoCo sim runner。它的用法是：

```bash
rl_sim_mujoco <robot_name> <scene_name>
```

對 Opus 來說，程式會找：

```text
/home/luis/rl_sar/src/rl_sar_zoo/opus_description/mjcf/<scene_name>.xml
```

也就是如果要跑 `scene`：

```text
/home/luis/rl_sar/src/rl_sar_zoo/opus_description/mjcf/scene.xml
```

目前已先從 Go2 copy 一份場景模板到：

```text
/home/luis/rl_sar/src/rl_sar_zoo/opus_description/mjcf/scene.xml
```

檢查：

```bash
find /home/luis/rl_sar/src/rl_sar_zoo/opus_description -maxdepth 3 -type f | sort | grep '/mjcf/'
```

注意：這份 `scene.xml` 是直接 copy Go2 的 scene，所以目前 include 仍是：

```text
<include file="go2.xml"/>
```

真正要跑 Opus 時，需要先準備 Opus 對應的 robot MJCF，並把 include 改成 Opus 的模型 XML；否則 `rl_sim_mujoco opus scene` 會找不到 `go2.xml` 或載入錯模型。

### Build MuJoCo target

MuJoCo library 目前應在：

```text
/home/luis/rl_sar/library/mujoco
```

檢查：

```bash
test -f /home/luis/rl_sar/library/mujoco/include/mujoco/mujoco.h && echo "mujoco header OK"
test -f /home/luis/rl_sar/library/mujoco/lib/libmujoco.so.3.2.7 && echo "mujoco lib OK"
```

編譯 `rl_sim_mujoco`：

```bash
cd /home/luis/rl_sar
source /opt/ros/humble/setup.bash
colcon build --merge-install --symlink-install --packages-select rl_sar --cmake-args -DUSE_MUJOCO=ON
source /home/luis/rl_sar/install/setup.bash
```

確認 executable：

```bash
ls -la /home/luis/rl_sar/install/bin/rl_sim_mujoco
ldd /home/luis/rl_sar/install/bin/rl_sim_mujoco | grep -E 'mujoco|glfw|not found'
```

如果看到 `not found`，先補：

```bash
export LD_LIBRARY_PATH=/home/luis/rl_sar/library/mujoco/lib:$LD_LIBRARY_PATH
```

### Run

假設已經有：

```text
/home/luis/rl_sar/src/rl_sar_zoo/opus_description/mjcf/scene.xml
```

啟動：

```bash
cd /home/luis/rl_sar
source /opt/ros/humble/setup.bash
source /home/luis/rl_sar/install/setup.bash
export LD_LIBRARY_PATH=/home/luis/rl_sar/library/mujoco/lib:$LD_LIBRARY_PATH

/home/luis/rl_sar/install/bin/rl_sim_mujoco opus scene
```

如果要跑其他場景，例如 `scene_terrain.xml`：

```bash
/home/luis/rl_sar/install/bin/rl_sim_mujoco opus scene_terrain
```

### MuJoCo controls

和 Gazebo FSM 基本一致：

```text
Num0 / A: get up
Num1: enter RL locomotion
Num9 / B: get down
P: passive
R: reset simulation
Enter: pause / unpause simulation
```

手柄預設讀：

```text
/dev/input/js0
```

如果沒有手柄，程式會打印 joystick open failed，但鍵盤仍可用。

### MuJoCo 注意事項

`rl_sim_mujoco` 的 IMU 和 joint sensor 依賴 MJCF sensor 排列：

```text
joint q sensors: 0 .. num_dofs-1
joint dq sensors: num_dofs .. 2*num_dofs-1
joint tau sensors: 2*num_dofs .. 3*num_dofs-1
quat + gyro sensors: after 3*num_dofs
```

所以 Opus 的 MJCF 必須有 12 個 joint position sensor、12 個 velocity sensor、12 個 torque/force sensor，後面再接 quaternion 和 gyro sensor。順序也必須和 `policy/opus/base.yaml` 的 raw joint order 對齊，否則 `joint_mapping` 會錯。

## 10. 風險點

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
