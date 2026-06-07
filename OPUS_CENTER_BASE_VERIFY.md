# Check Opus centered base URDF

This checks the updated `opus_description` URDF after moving the base frame to the robot center.

Updated package:

```bash
/home/luis/rl_sar/src/rl_sar_zoo/opus_description
```

Latest source zip:

```bash
/home/luis/Downloads/opus_urdf_final_v2-20260530T064406Z-3-001.zip
```

## 1. Check files

```bash
PKG=/home/luis/rl_sar/src/rl_sar_zoo/opus_description

ls -la "$PKG"
find "$PKG" -maxdepth 3 -type f | sort
```

## 2. Check centered base values

```bash
PKG=/home/luis/rl_sar/src/rl_sar_zoo/opus_description
URDF="$PKG/urdf/opus_description.urdf"

rg -n '<link name="base"|<joint name="(FR|FL|RR|RL)_hip_joint"|<origin xyz="(-5\.988|0\.144|-0\.144)' "$URDF"
```

Expected important values:

```text
base inertial origin: -5.9883897840507E-10 -1.27852173292808E-11 0.0468319439148929
FR_hip_joint origin: 0.144000000000001 -0.0499998874435837 0.0482497282672308
FL_hip_joint origin: 0.144000000000001 0.0500001125564163 0.0482497282672306
RR_hip_joint origin: -0.144000000000003 -0.0500001125564116 0.0482497282672306
RL_hip_joint origin: -0.144000000000003 0.0499998874435886 0.0482497282621224
```

## 3. Check base mesh hashes

```bash
PKG=/home/luis/rl_sar/src/rl_sar_zoo/opus_description
ZIP=/home/luis/Downloads/opus_urdf_final_v2-20260530T064406Z-3-001.zip

sha256sum "$PKG/meshes/base_link.STL"
unzip -p "$ZIP" opus_urdf_final_v2/opus_final/meshes/base_link.STL | sha256sum

sha256sum "$PKG/meshes/collision/base_link.STL"
unzip -p "$ZIP" opus_urdf_final_v2/opus_urdf_collision/meshes/base_link.STL | sha256sum
```

Expected hashes:

```text
visual base_link.STL:    2bb3a3f4ebe406658621c274ce1330dcf7ff6f6fb2b49a23d0e64a89ad730d56
collision base_link.STL: 079f9ca276963017e1605dd8ec91e7eaaf27c38176ed2110f4fa912d3edc1fa9
```

## 4. Parse URDF

```bash
source /opt/ros/humble/setup.bash

PKG=/home/luis/rl_sar/src/rl_sar_zoo/opus_description
URDF="$PKG/urdf/opus_description.urdf"

check_urdf "$URDF"
```

Expected result includes:

```text
Successfully Parsed XML
root Link: base has 4 child(ren)
```

## 5. Check mesh paths and important names

```bash
PKG=/home/luis/rl_sar/src/rl_sar_zoo/opus_description

rg -n 'package://|file://|meshes/collision|<sphere|<link name=|<joint name=|<collision>|<origin' "$PKG/urdf"
```

## 6. Build package

```bash
source /opt/ros/humble/setup.bash

cd /home/luis/rl_sar
colcon build --merge-install --symlink-install --packages-select opus_description
source install/setup.bash
```

## 7. Create temporary file-path URDF for RViz2

Use this if RViz2 says the package does not exist.

```bash
PKG=/home/luis/rl_sar/src/rl_sar_zoo/opus_description
URDF="$PKG/urdf/opus_description.urdf"
TMP=/tmp/opus_center_base_file.urdf

sed 's#package://opus_description#file:///home/luis/rl_sar/src/rl_sar_zoo/opus_description#g' \
  "$URDF" > "$TMP"

check_urdf "$TMP"
```

## 8. RViz2 check - terminal 1

```bash
source /opt/ros/humble/setup.bash
source /home/luis/rl_sar/install/setup.bash

URDF=/tmp/opus_center_base_file.urdf

ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat "$URDF")"
```

Keep this terminal running.

## 9. RViz2 check - terminal 2

```bash
source /opt/ros/humble/setup.bash
source /home/luis/rl_sar/install/setup.bash

ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

Keep this terminal running.

## 10. RViz2 check - terminal 3

```bash
source /opt/ros/humble/setup.bash
source /home/luis/rl_sar/install/setup.bash

rviz2
```

RViz2 settings:

```text
Global Options -> Fixed Frame = base

Add -> RobotModel
RobotModel -> Description Source = Topic
RobotModel -> Description Topic = /robot_description
RobotModel -> Visual Enabled = true
RobotModel -> Collision Enabled = true
```

## 11. If RViz2 is red

```bash
source /opt/ros/humble/setup.bash
source /home/luis/rl_sar/install/setup.bash

ros2 topic list | grep -E '^/(robot_description|tf|tf_static)$'
ros2 topic echo /robot_description --once
ros2 run tf2_ros tf2_echo base FR_foot
```

If `FR_foot` does not exist, list the actual frames:

```bash
ros2 run tf2_tools view_frames
```
