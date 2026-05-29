# OPUS IMU RViz2 Verification

The visualizer and `rl_real_opus.cpp` both read:

```text
/home/luis/rl_sar/src/rl_sar/library/thirdparty/test_motor_thread/bin/imu_interface.py -> imu_receiver.py
```

So `/opus_imu/data` is the same IMU data chain used by the real OPUS script.

## 1. Start The Visualizer

```bash
source /opt/ros/humble/setup.bash
python3 /home/luis/rl_sar/src/rl_sar/scripts/imu_rviz2_visualizer.py
```

## 2. Open RViz2

Use another terminal:

```bash
source /opt/ros/humble/setup.bash
rviz2
```

Set `Fixed Frame` to:

```text
world
```

Add these displays:

```text
TF
MarkerArray: /opus_imu/axes
Imu: /opus_imu/data
```

## 3. Check The Published IMU Message

Use another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /opus_imu/data
```

Expected data convention after the receiver change:

```text
orientation:
  q_ros = [w, x, -y, -z] from raw FDI [w, x, y, z]

angular_velocity:
  x =  gyro_x
  y = -gyro_y
  z = -gyro_z

linear_acceleration:
  x =  accel_x
  y = -accel_y
  z = -accel_z
```

## 4. Quick Motion Checks

Move the IMU slowly and check RViz2:

```text
X arrow: forward
Y arrow: left
Z arrow: up
```

The terminal running `imu_rviz2_visualizer.py` also prints quaternion and gyro once per second.
