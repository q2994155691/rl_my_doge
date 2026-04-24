/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef OPUS_FSM_HPP
#define OPUS_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"

namespace opus_fsm
{

inline void LogTarget(const char *phase, const RobotCommand<float> *fsm_command, const RobotState<float> *fsm_state)
{
    static auto last_log = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_log).count() >= 0.2)
    {
        last_log = now;
        std::cout << LOGGER::INFO
                  << phase
                  << " tgt[0]=" << std::fixed << std::setprecision(3) << fsm_command->motor_command.q[0]
                  << " [1]=" << fsm_command->motor_command.q[1]
                  << " [2]=" << fsm_command->motor_command.q[2]
                  << " | cur[0]=" << fsm_state->motor_state.q[0]
                  << " [1]=" << fsm_state->motor_state.q[1]
                  << " [2]=" << fsm_state->motor_state.q[2]
                  << " | dq[0]=" << fsm_state->motor_state.dq[0]
                  << " [1]=" << fsm_state->motor_state.dq[1]
                  << " [2]=" << fsm_state->motor_state.dq[2]
                  << std::endl;
    }
}

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
    }

    void Run() override
    {
        static auto last_log = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_log).count() >= 1.0)
        {
            last_log = now;
            int n = rl.params.Get<int>("num_of_dofs");
            std::cout << LOGGER::INFO << "Passive encoder q (logical):";
            for (int i = 0; i < n; ++i)
            {
                std::cout << " [" << i << "]=" << std::fixed << std::setprecision(4) << fsm_state->motor_state.q[i];
            }
            std::cout << std::endl;
        }

        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            // fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 0.05;
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    float percent_pre_getup = 0.0f;
    float percent_getup = 0.0f;
    std::vector<float> pre_running_pos = {
        0.00, 1.36, -2.65,
        0.00, 1.36, -2.65,
        0.00, 1.36, -2.65,
        0.00, 1.36, -2.65
    };
    bool stand_from_passive = true;

    void Enter() override
    {
        percent_pre_getup = 0.0f;
        percent_getup = 0.0f;
        if (rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive")
        {
            stand_from_passive = true;
        }
        else
        {
            stand_from_passive = false;
        }
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        if(stand_from_passive)
        {

            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 1.0f, "Pre Getting up", true)) { LogTarget("PreGetUp", fsm_command, fsm_state); return; }
            if (Interpolate(percent_getup, pre_running_pos, rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f, "Getting up", true)) { LogTarget("GetUp", fsm_command, fsm_state); return; }
        }
        else
        {
            if (Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 1.0f, "Getting up", true)) { LogTarget("GetUp", fsm_command, fsm_state); return; }
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRLLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num2)
            {
                return "RLFSMStateSineTest";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num3)
            {
                return "RLFSMStateTrot";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 2.0f, "Getting down", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X || percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateSineTest : public RLFSMState
{
public:
    RLFSMStateSineTest(RL *rl) : RLFSMState(*rl, "RLFSMStateSineTest") {}

    float amplitude = 0.3f;   // rad
    float frequency = 0.5f;   // Hz
    float elapsed_time = 0.0f;
    std::vector<float> default_pos;

    void Enter() override
    {
        elapsed_time = 0.0f;
        default_pos = rl.params.Get<std::vector<float>>("default_dof_pos");
        std::cout << LOGGER::NOTE << "Entered Sine Test mode. Press 'P' to exit, '0' to GetUp." << std::endl;
    }

    void Run() override
    {
        float dt = rl.params.Get<float>("dt");
        elapsed_time += dt;
        float phase = 2.0f * M_PI * frequency * elapsed_time;

        int num_dofs = rl.params.Get<int>("num_of_dofs");
        auto fixed_kp = rl.params.Get<std::vector<float>>("fixed_kp");
        auto fixed_kd = rl.params.Get<std::vector<float>>("fixed_kd");

        for (int i = 0; i < num_dofs; ++i)
        {
            float offset = amplitude * std::sin(phase);
            fsm_command->motor_command.q[i]   = default_pos[i] + offset;
            fsm_command->motor_command.dq[i]  = 0;
            fsm_command->motor_command.kp[i]  = fixed_kp[i];
            fsm_command->motor_command.kd[i]  = fixed_kd[i];
            fsm_command->motor_command.tau[i] = 0;
        }

        // Log every 0.5s
        static auto last_log = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_log).count() >= 0.5)
        {
            last_log = now;
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                      << "SineTest t=" << std::fixed << std::setprecision(2) << elapsed_time
                      << "s q[0]=" << std::setprecision(3) << fsm_command->motor_command.q[0]
                      << " q[1]=" << fsm_command->motor_command.q[1]
                      << " q[2]=" << fsm_command->motor_command.q[2] << std::flush;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateTrot : public RLFSMState
{
public:
    RLFSMStateTrot(RL *rl) : RLFSMState(*rl, "RLFSMStateTrot") {}

    float gait_phase = 0.0f;
    const float phase_offset[4] = {0.0f, 0.5f, 0.5f, 0.0f}; // FR, FL, RR, RL (diagonal sync)
    const float gait_freq = 2.0f;   // Hz
    const float swing_amp = 0.2f;   // rad

    void Enter() override
    {
        gait_phase = 0.0f;
        std::cout << LOGGER::NOTE << "Entered Trot mode. Press 'P' to pause, '9' to get down." << std::endl;
    }

    void Run() override
    {
        float dt = rl.params.Get<float>("dt");
        gait_phase = std::fmod(gait_phase + gait_freq * dt, 1.0f);

        auto default_pos = rl.params.Get<std::vector<float>>("default_dof_pos");
        auto kp = rl.params.Get<std::vector<float>>("fixed_kp");
        auto kd = rl.params.Get<std::vector<float>>("fixed_kd");
        int n = rl.params.Get<int>("num_of_dofs");

        for (int leg = 0; leg < 4; ++leg)
        {
            float lp = std::fmod(gait_phase + phase_offset[leg], 1.0f);
            float s = std::sin(2.0f * M_PI * lp);
            int b = leg * 3;

            if (b + 0 < n) fsm_command->motor_command.q[b + 0] = default_pos[b + 0];
            if (b + 1 < n) fsm_command->motor_command.q[b + 1] = default_pos[b + 1] + swing_amp * s;
            if (b + 2 < n) fsm_command->motor_command.q[b + 2] = default_pos[b + 2] - swing_amp * s;

            for (int j = 0; j < 3; ++j)
            {
                int idx = b + j;
                if (idx >= n) break;
                fsm_command->motor_command.dq[idx]  = 0.0f;
                fsm_command->motor_command.tau[idx] = 0.0f;
                fsm_command->motor_command.kp[idx]  = kp[idx];
                fsm_command->motor_command.kd[idx]  = kd[idx];
            }
        }

        LogTarget("Trot", fsm_command, fsm_state);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        return state_name_;
    }
};

class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "legged_gym";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }
        return state_name_;
    }
};

} // namespace opus_fsm

class OpusFSMFactory : public FSMFactory
{
public:
    OpusFSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<opus_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<opus_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<opus_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateSineTest")
            return std::make_shared<opus_fsm::RLFSMStateSineTest>(rl);
        else if (state_name == "RLFSMStateTrot")
            return std::make_shared<opus_fsm::RLFSMStateTrot>(rl);
        else if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<opus_fsm::RLFSMStateRLLocomotion>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "opus"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateSineTest",
            "RLFSMStateTrot",
            "RLFSMStateRLLocomotion"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(OpusFSMFactory, "RLFSMStatePassive")

#endif // OPUS_FSM_HPP
