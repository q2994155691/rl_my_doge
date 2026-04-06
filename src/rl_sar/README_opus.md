# RL_Opus 開發筆記

## 目標
將 `test_motor_thread` 的 IMU 線程整合進 `rl_sar` 框架，建立一個新的機器人 class `RL_Opus`，繼承 `RL` base class，override `GetState` / `SetCommand` / `Forward`。

## 相關檔案
| 檔案 | 說明 |
|------|------|
| `src/rl_real_opus.cpp` | 主實作 |
| `include/rl_real_opus.hpp` | class 定義 |
| `fsm_robot/fsm_opus.hpp` | FSM states（基於 fsm_a1） |
| `~/test_motor_thread/include/user/IMUReader.h` | IMU 讀取（Python 嵌入） |
| `~/test_motor_thread/include/unitree/g1/` | DataBuffer / MotorState / BaseState |

## 目前狀態（第一版）
- IMU 線程：已接上，200Hz，從 `base_state_buffer_` 讀資料填進 `RobotState::imu`
- Motor state：填假值（全 0），motor thread 尚未接上
- SetCommand：寫進 `motor_command_buffer_`，目前只 log，不送硬體
- FSM：Passive → GetUp → RLLocomotion 流程完整
- yaml config：`policy/opus/base.yaml` 尚未建立

## 已知問題：Ctrl+C 中止不了程序

### 症狀
按 Ctrl+C 後，loops 正常 shutdown，但卡在 `Py_Finalize()` 不返回。

### 根本原因（推測）
Python GIL deadlock：
1. IMU thread 在 `fetchIMUData()` 裡透過 `PyGILState_Ensure()` 持有 GIL
2. 主線程的 `IMUReader` 析構呼叫 `Py_Finalize()`，同樣需要 GIL
3. 兩者互等 → deadlock

### 目前嘗試的修法
- `imu_python_mutex_`：在 `IMUStateReader()` 裡 lock，析構時 lock 確保 fetch 跑完才析構
- `PyGILState_Ensure()` 在 `Py_Finalize()` 前取得 GIL

### 仍然卡住的可能原因
`RecurrentThread` 沒有 join，析構時雖然 mutex 確保當前 fetch 完成，但 thread 本身還活著，下一個 `PyGILState_Ensure()` 可能在 `Py_Finalize()` 執行途中被呼叫，造成 Python 內部狀態損毀或再次 deadlock。

### 下一步
考慮完全不呼叫 `Py_Finalize()`，讓 OS 在 process 結束時自動清理所有資源（fd 實驗已確認 OS 會回收），避免和仍在執行的 IMU thread 競爭。
