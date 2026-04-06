/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_OPUS_HPP
#define RL_REAL_OPUS_HPP

// #define PLOT
// #define CSV_LOGGER
// #define USE_ROS

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_opus.hpp"

#include "unitree/common/thread/recurrent_thread.hpp"
#include "unitree/g1/data_buffer.hpp"
#include "unitree/g1/motors.hpp"
#include "unitree/g1/base_state.hpp"
#include "user/IMUReader.h"
#include "user/Motor_thread.hpp"

#include <csignal>

using namespace unitree::common;

class RL_Opus : public RL
{
public:
    RL_Opus();
    ~RL_Opus();

private:
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    // imu thread
    void IMUStateReader();
    void RecordBaseState();

    // motor thread
    void JointStateReadWriter();
    void RecordMotorState(const std::array<MotorData, 10> &data);
    unitree_hg::msg::dds_::LowCmd_ SetMotorCmd();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;

    // imu
    std::mutex imu_python_mutex_;
    volatile bool imu_running_ = true;
    IMUReader imu_reader_;
    ThreadPtr imu_thread_ptr_;

    // motor
    MotorController motor_ctrl_;
    ThreadPtr motor_thread_ptr_;

    // buffers
    DataBuffer<BaseState>    base_state_buffer_;
    DataBuffer<MotorState>   motor_state_buffer_;
    DataBuffer<MotorCommand> motor_command_buffer_;
};

#endif // RL_REAL_OPUS_HPP
