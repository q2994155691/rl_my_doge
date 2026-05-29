# OPUS Porting Status

Date: 2026-04-27

## Context

OPUS is being integrated into `rl_sar` using the motor and IMU thread code ported from the Qmini project:

- OPUS FSM: `src/rl_sar/fsm_robot/fsm_opus.hpp`
- OPUS real runner: `src/rl_sar/src/rl_real_opus.cpp`
- OPUS real header: `src/rl_sar/include/rl_real_opus.hpp`
- Ported motor/IMU code: `src/rl_sar/library/thirdparty/test_motor_thread`
- Qmini reference repository: `/home/luis/RoboTamerSdk4Qmini`

OPUS has the same quadruped layout family as Unitree A1/Go2: 4 legs, 3 motors per leg, 12 DOF total. Qmini is different, so the ported electrical-control logic must be adjusted gradually to match `rl_sar`'s quadruped assumptions.

## Current Working Tree

The repo currently has modifications in:

- `policy/opus/base.yaml`
- `src/rl_sar/include/rl_real_opus.hpp`
- `src/rl_sar/src/rl_real_opus.cpp`
- `src/rl_sar/library/thirdparty/test_motor_thread/include/unitree/g1/motors.hpp`
- `src/rl_sar/library/thirdparty/test_motor_thread/include/user/Motor_thread.hpp`

There is also an untracked `.codex` file in the repo root.

## Confirmed Current Changes

### OPUS DOF Count

`policy/opus/base.yaml` has been changed from:

```yaml
num_of_dofs: 10
```

to:

```yaml
num_of_dofs: 12
```

`src/rl_sar/library/thirdparty/test_motor_thread/include/unitree/g1/motors.hpp` has been changed from:

```cpp
const int G1_NUM_MOTOR = 10;
```

to:

```cpp
const int G1_NUM_MOTOR = 12;
```

These two changes were made deliberately as the first agreed step: changing OPUS from 10 DOF to 12 DOF.

### RecordMotorState Is Already 12 DOF

The following files already use 12-axis motor data:

- `src/rl_sar/include/rl_real_opus.hpp`
- `src/rl_sar/src/rl_real_opus.cpp`

Current shape:

```cpp
RecordMotorState(const std::array<MotorData, 12> &data)
```

and the loop records 12 motors.

This change existed in the working tree before the latest DOF-count step.

### Existing OPUS Pose / Sign Changes

`policy/opus/base.yaml` currently has customized `default_dof_pos` values, not the original all-A1-style defaults.

`Motor_thread.hpp` currently has a `Sign` entry where the last leg starts with `1` instead of `+1`. This is numerically identical, but appears as a diff because of formatting.

These are treated as existing user-side changes and should not be reverted unless explicitly requested.

## Confirmed Removed / Not Present

The previous experimental changes are not currently present:

- No `cmd_snapshot` logic in `Motor_thread.hpp`
- No `std::runtime_error` config checks added to `rl_real_opus.cpp`
- No `joint_mapping` remapping added inside `RL_Opus::GetState()`
- No `joint_mapping` remapping added inside `RL_Opus::SetMotorCmd()`
- No extra command mutex read-lock change in the motor loop

## Important Policy For Future Edits

Future code changes should be done one step at a time:

1. Inspect and discuss the exact intended change.
2. List the exact files and behavior to change.
3. Wait for explicit approval before editing code.
4. Apply only that approved change.
5. Re-check diff after each step.

## Next Items To Discuss

These are likely needed, but should not be changed until discussed and approved:

- Whether `joint_mapping` should be applied in OPUS real hardware read/write paths.
- Whether OPUS hardware motor order exactly matches `FR, FL, RR, RL` with `[hip, thigh, calf]` per leg.
- Whether `Startq` values in `Motor_thread.hpp` match the true OPUS zero pose.
- Whether `Sign` values match the physical motor installation directions.
- Whether OPUS needs a real `policy/opus/legged_gym/config.yaml` or should use another policy config name.
- Whether `fixed_kp`, `fixed_kd`, and RL gains should be raised from the current low test values.
