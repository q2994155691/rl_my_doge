# GitHub Build And Push Guide

## 1. Clone With Submodules

This project uses git submodules for third-party robot SDKs.

```bash
git clone --recurse-submodules https://github.com/q2994155691/rl_my_doge.git
cd rl_my_doge
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

## 2. Install System Dependencies

Ubuntu base dependencies:

```bash
sudo apt update
sudo apt install -y \
  cmake g++ build-essential \
  libyaml-cpp-dev libeigen3-dev libboost-all-dev \
  libspdlog-dev libfmt-dev libtbb-dev liblcm-dev
```

ROS2 dependencies, for example Humble:

```bash
sudo apt install -y \
  ros-humble-teleop-twist-keyboard \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-control-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-rviz2
```

Python serial dependency for the FDIlink IMU reader:

```bash
sudo apt install -y python3-serial
```

## 3. ROS2 Build

Use merge install:

```bash
cd /home/luis/rl_sar
source /opt/ros/humble/setup.bash
colcon build --merge-install --symlink-install
source install/setup.bash
```

Expected result:

```text
Summary: 15 packages finished
```

Warnings about deprecated Python C API calls are not build failures.

## 4. Hardware CMake Build

For hardware deployment without ROS simulation:

```bash
cd /home/luis/rl_sar
./build.sh -m
```

The executable output is under:

```text
cmake_build/bin/
```

## 5. OPUS IMU Data Path

The OPUS real runner now imports the IMU Python code from inside this repository:

```text
src/rl_sar/library/thirdparty/test_motor_thread/bin/imu_interface.py
src/rl_sar/library/thirdparty/test_motor_thread/bin/imu_receiver.py
```

The receiver applies the FDIlink ROS `device_type=1` convention:

```text
gyro  = [gx, -gy, -gz]
accel = [ax, -ay, -az]
quat  = [w, x, -y, -z]
```

## 6. Check Before Pushing

Review the working tree:

```bash
cd /home/luis/rl_sar
git status --short
git diff --stat
```

Check large tracked files:

```bash
git ls-files | xargs -r du -h 2>/dev/null | sort -hr | sed -n '1,40p'
```

Check untracked files before adding everything:

```bash
git status --short
```

Do not accidentally commit local-only cache/build outputs such as:

```text
build/
install/
log/
cmake_build/
__pycache__/
```

## 7. Push To GitHub

Current remotes:

```text
origin   https://github.com/q2994155691/rl_my_doge.git
upstream https://github.com/fan-ziqi/rl_sar.git
```

Current branch:

```text
main
```

Recommended commands:

```bash
cd /home/luis/rl_sar

git status --short
git add .gitmodules \
  GITHUB_BUILD_AND_PUSH.md \
  OPUS_IMU_RVIZ2_VERIFY.md \
  A1_IMU_RVIZ2_COMMANDS.md \
  policy/opus \
  src/rl_sar \
  policy/a1/base.yaml \
  policy/opus/base.yaml

git status --short
git diff --cached --stat
git commit -m "Add OPUS hardware IMU integration and build notes"
git push origin main
```

If you intentionally want to include every current change and untracked file:

```bash
cd /home/luis/rl_sar
git add -A
git status --short
git diff --cached --stat
git commit -m "Add OPUS hardware integration"
git push origin main
```

Use `git add -A` only after confirming local notes, backups, and deleted files are intended.
