# SBUS FSM Remote Test Commands

## 1. Dry-run channel mapping

Use this first to confirm the receiver channel values before enabling the built-in FSM remote.

```bash
cd /home/luis/rl_sar
python3 src/rl_sar/scripts/sbus_joy_remote.py --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --dry-run
```

Expected mapping:

```text
CH2 -> x
CH4 -> y
CH1 -> yaw
CH5 raw low, around 200 -> stand_up
CH5 raw high, around 1800 -> rl_mode
CH6 raw low, around 200 -> get_down
CH6 raw high, around 1800 -> passive
```

## 2. Enable built-in FSM remote

Edit `/home/luis/rl_sar/policy/opus/base.yaml`:

```yaml
sbus_remote_enabled: true
sbus_remote_port: "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
```

Keep it `false` when using keyboard/gamepad.

## 3. Rebuild after code changes

```bash
cd /home/luis/rl_sar
cmake --build build/rl_sar --target rl_sim -j2
```

For real OPUS:

```bash
cd /home/luis/rl_sar
cmake --build build/rl_sar --target rl_real_opus -j2
```

## 4. Run ROS2 sim

```bash
cd /home/luis/rl_sar
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=opus
```

When `sbus_remote_enabled: true`, OPUS FSM reads SBUS directly. Do not run the Python `/joy` publisher at the same time.

cd /home/luis/rl_sar
source /opt/ros/humble/setup.bash
source /home/luis/rl_sar/install/setup.bash
ros2 run rl_sar rl_sim

## 5. Direction fixes

Edit `/home/luis/rl_sar/policy/opus/base.yaml`. YAML changes are read at startup, so restart the program after editing.

```yaml
sbus_remote_invert_x: true
sbus_remote_invert_y: false
sbus_remote_invert_yaw: false
```

## 6. Wider center deadband

Use this if the sticks jitter around center.

```yaml
sbus_remote_deadband: 40
```
