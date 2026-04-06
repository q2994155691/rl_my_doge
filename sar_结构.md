# RL-SAR 项目结构

```
rl_sar/
├── policy/                                    # 训练好的策略文件
│   ├── a1/
│   │   ├── base.yaml                          # 机器人硬件参数（关节数、kp/kd、观测配置）
│   │   ├── legged_gym/
│   │   │   ├── config.yaml                    # 策略配置（obs缩放、clip范围）
│   │   │   └── policy.pt / policy.onnx        # 神经网络模型
│   │   └── robot_lab/
│   │       └── config.yaml
│   ├── go2/ go2w/ b2/ b2w/                    # Unitree 系列机器人
│   ├── g1/ gr1t1/ gr1t2/                      # 人形机器人
│   └── lite3/ l4w4/ tita/                     # 其他机器人
│
├── src/
│   ├── rl_sar/                                # 核心包
│   │   ├── CMakeLists.txt                     # 支持 ROS1/ROS2/CMAKE 三种编译模式
│   │   │
│   │   ├── library/core/                      # 共享核心库
│   │   │   ├── rl_sdk/                        # RL基类：观测计算、推理、输出转换、状态机、安全保护
│   │   │   │   ├── rl_sdk.hpp
│   │   │   │   └── rl_sdk.cpp
│   │   │   ├── fsm/                           # 有限状态机框架（RESET/STAND/WALK/SKILL）
│   │   │   │   └── fsm.hpp
│   │   │   ├── inference_runtime/             # 统一推理接口（LibTorch/ONNX）
│   │   │   │   ├── inference_runtime.hpp
│   │   │   │   └── inference_runtime.cpp
│   │   │   ├── observation_buffer/            # 历史观测滑动窗口
│   │   │   │   ├── observation_buffer.hpp
│   │   │   │   └── observation_buffer.cpp
│   │   │   ├── motion_loader/                 # 插值运动加载（站立/预备动作）
│   │   │   │   ├── motion_loader.hpp
│   │   │   │   └── motion_loader.cpp
│   │   │   ├── loop/                          # 高精度定时循环
│   │   │   │   └── loop.hpp
│   │   │   ├── logger/                        # 彩色日志输出
│   │   │   │   └── logger.hpp
│   │   │   ├── vector_math/                   # 数学工具（clamp等）
│   │   │   │   └── vector_math.hpp
│   │   │   └── matplotlibcpp/                 # 绘图库（第三方）
│   │   │       └── matplotlibcpp.h
│   │   │
│   │   ├── fsm_robot/                         # 各机器人状态机定义
│   │   │   ├── fsm_all.hpp                    # 注册所有机器人FSM
│   │   │   ├── fsm_a1.hpp
│   │   │   ├── fsm_go2.hpp / fsm_go2w.hpp
│   │   │   ├── fsm_b2.hpp / fsm_b2w.hpp
│   │   │   ├── fsm_g1.hpp
│   │   │   ├── fsm_gr1t1.hpp / fsm_gr1t2.hpp
│   │   │   ├── fsm_lite3.hpp
│   │   │   ├── fsm_l4w4.hpp
│   │   │   └── fsm_tita.hpp
│   │   │
│   │   ├── include/                           # 平台特定头文件
│   │   │   ├── rl_sim.hpp                     # Gazebo仿真（ROS1/ROS2）
│   │   │   ├── rl_sim_mujoco.hpp              # MuJoCo仿真（独立运行）
│   │   │   ├── rl_real_a1.hpp                 # A1实机（Unitree UDP）
│   │   │   ├── rl_real_go2.hpp                # Go2实机（Unitree DDS）
│   │   │   ├── rl_real_g1.hpp                 # G1实机（Unitree DDS）
│   │   │   ├── rl_real_lite3.hpp              # Lite3实机（DeepRobotics SDK）
│   │   │   └── rl_real_l4w4.hpp               # L4W4实机（自定义SDK）
│   │   │
│   │   ├── src/                               # 平台特定实现
│   │   │   ├── rl_sim.cpp
│   │   │   ├── rl_sim_mujoco.cpp
│   │   │   ├── rl_real_a1.cpp
│   │   │   ├── rl_real_go2.cpp
│   │   │   ├── rl_real_g1.cpp
│   │   │   ├── rl_real_lite3.cpp
│   │   │   └── rl_real_l4w4.cpp
│   │   │
│   │   ├── launch/
│   │   │   └── gazebo.launch.py               # 启动Gazebo+控制器
│   │   │
│   │   ├── scripts/
│   │   │   ├── convert_policy.py              # 模型格式转换（.pt ↔ .onnx）
│   │   │   └── actuator_net.py                # 执行器网络工具
│   │   │
│   │   └── test/                              # 单元测试
│   │       ├── test_inference_runtime.cpp
│   │       ├── test_observation_buffer.cpp
│   │       └── test_vector_math.cpp
│   │
│   ├── rl_sar_zoo/                            # 各机器人描述文件
│   │   ├── a1_description/
│   │   │   ├── urdf/                          # URDF模型（Gazebo）
│   │   │   ├── mjcf/                          # MuJoCo场景
│   │   │   ├── meshes/                        # 3D网格文件
│   │   │   ├── config/
│   │   │   │   ├── robot_control.yaml         # ROS1控制器配置
│   │   │   │   └── robot_control_ros2.yaml    # ROS2控制器配置
│   │   │   └── launch/
│   │   │       └── gazebo.launch.py
│   │   ├── go2_description/ go2w_description/
│   │   ├── b2_description/ b2w_description/
│   │   ├── g1_description/                    # 含多种DOF配置（23/29dof）
│   │   ├── gr1t1_description/ gr1t2_description/
│   │   ├── lite3_description/ l4w4_description/
│   │   └── tita_description/
│   │
│   ├── robot_joint_controller/                # ROS关节控制器插件（Gazebo PD控制）
│   │   ├── ros/src/                           # ROS1实现
│   │   └── ros2/src/                          # ROS2实现
│   │
│   └── robot_msgs/                            # 自定义ROS消息
│       └── msg/                               # MotorState, MotorCommand, RobotState, RobotCommand
│
└── library/                                   # 第三方库
    └── inference_runtime/
        ├── libtorch/                          # LibTorch（PyTorch C++）
        └── onnxruntime/                       # ONNX Runtime（可选）
```

## 核心模块功能

### rl_sdk
- **观测计算**：传感器数据 → 策略网络输入（坐标系转换、归一化、历史缓冲）
- **异步推理**：独立线程运行神经网络，无锁队列传递结果
- **输出转换**：策略输出 → 电机指令（clip、关节映射、PD控制）
- **状态机集成**：FSM状态更新 + 键盘/手柄控制
- **安全保护**：力矩限幅、姿态保护（防翻倒）

### fsm
- 有限状态机框架：RESET（复位）、PASSIVE（被动）、STAND（站立）、WALK（行走）、SKILL1-7（自定义技能）
- 状态生命周期：OnEnter() → Run() → OnExit()

### inference_runtime
- 统一推理接口，封装 LibTorch 和 ONNX Runtime
- 自动检测模型格式（.pt / .onnx）

### observation_buffer
- 维护固定长度的观测历史（如最近50帧）
- 用于时序策略（LSTM/Transformer）

### motion_loader
- 加载预定义关节轨迹（站立姿态、预备动作）
- 提供平滑插值（线性/三次样条）

### fsm_robot
- 各机器人的状态机实现（继承自 RLFSMState）
- 定义状态转换逻辑和特定控制策略

### 平台实现（include/ + src/）
- **rl_sim**：Gazebo仿真（ROS1/ROS2）
- **rl_sim_mujoco**：MuJoCo仿真（独立运行）
- **rl_real_***：各机器人实机（Unitree UDP/DDS、DeepRobotics SDK、自定义SDK）

### robot_joint_controller
- Gazebo插件，接收 RobotCommand 消息
- 执行 PD 控制：`tau = kp*(q_target - q) + kd*(dq_target - dq) + tau_ff`
- 发布 RobotState 消息

### rl_sar_zoo
- 各机器人的 URDF/MJCF 模型、网格文件、控制器配置、启动文件
