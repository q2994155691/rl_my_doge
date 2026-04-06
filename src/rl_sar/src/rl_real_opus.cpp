/*
* Copyright (c) 2024-2025 Ziqi Fan
* SPDX-License-Identifier: Apache-2.0
*/

#include "rl_real_opus.hpp"

RL_Opus::RL_Opus()
{
    // read params from yaml
    this->ang_vel_axis = "body";
    this->robot_name = "opus";
    this->ReadYaml(this->robot_name, "base.yaml");

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

    // imu thread
    PyEval_ReleaseLock();
    this->imu_thread_ptr_ = CreateRecurrentThreadEx("imu", UT_CPU_ID_NONE, 0.005 * 1e6, &RL_Opus::IMUStateReader, this);
    usleep(0.1 * 1e6);

    // motor thread (500Hz)
    this->motor_thread_ptr_ = CreateRecurrentThreadEx("motor", UT_CPU_ID_NONE, 0.002 * 1e6, &RL_Opus::JointStateReadWriter, this);
    usleep(0.1 * 1e6);

    // loop
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05,  std::bind(&RL_Opus::KeyboardInterface, this));
    this->loop_control  = std::make_shared<LoopFunc>("loop_control",  this->params.Get<float>("dt"), std::bind(&RL_Opus::RobotControl, this));
    this->loop_rl       = std::make_shared<LoopFunc>("loop_rl",       this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Opus::RunModel, this));
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();
}

RL_Opus::~RL_Opus()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    this->motor_thread_ptr_.reset();
    this->imu_thread_ptr_.reset();
    std::cout << LOGGER::INFO << "RL_Opus exit" << std::endl;
}

void RL_Opus::GetState(RobotState<float> *state)
{
    // 從 base_state_buffer_ 讀 IMU
    auto bs = base_state_buffer_.GetData();
    if (bs)
    {
        state->imu.quaternion[0] = bs->quat[0]; // w
        state->imu.quaternion[1] = bs->quat[1]; // x
        state->imu.quaternion[2] = bs->quat[2]; // y
        state->imu.quaternion[3] = bs->quat[3]; // z

        for (int i = 0; i < 3; ++i)
        {
            state->imu.gyroscope[i] = bs->omega[i];
        }

    }

    // motor state from buffer
    auto ms = motor_state_buffer_.GetData();
    if (ms)
    {
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            state->motor_state.q[i]       = ms->q[i];
            state->motor_state.dq[i]      = ms->dq[i];
            state->motor_state.tau_est[i] = ms->tau_est[i];
        }
    }
}

void RL_Opus::SetCommand(const RobotCommand<float> *command)
{
    MotorCommand mc;
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        mc.q_target[i]  = command->motor_command.q[i];
        mc.dq_target[i] = command->motor_command.dq[i];
        mc.kp[i]        = command->motor_command.kp[i];
        mc.kd[i]        = command->motor_command.kd[i];
        mc.tau_ff[i]    = command->motor_command.tau[i];
    }
    this->motor_command_buffer_.SetData(mc);
    //std::cout << "[SetCommand] q[0]=" << mc.q_target[0] << " kp[0]=" << mc.kp[0] << " kd[0]=" << mc.kd[0] << std::endl;
}

void RL_Opus::RobotControl()
{
    this->GetState(&this->robot_state);
    this->StateController(&this->robot_state, &this->robot_command);
    this->control.ClearInput();
    this->SetCommand(&this->robot_command);
}

void RL_Opus::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
#if !defined(USE_CMAKE) && defined(USE_ROS)
        if (this->control.navigation_mode)
        {
            this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};

        }
#endif
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Opus::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Opus::IMUStateReader()
{
    std::lock_guard<std::mutex> lock(imu_python_mutex_);
    if (!imu_running_) return;
    if (imu_reader_.fetchIMUData())
    {
        RecordBaseState();
    }
}

void RL_Opus::RecordBaseState()
{
    BaseState bs_tmp;
    bs_tmp.omega = {imu_reader_.RollSpeed, imu_reader_.PitchSpeed, imu_reader_.HeadingSpeed};
    bs_tmp.rpy = {imu_reader_.Roll, imu_reader_.Pitch, imu_reader_.Heading};
    bs_tmp.acc = {imu_reader_.Accelerometer_X, imu_reader_.Accelerometer_Y, imu_reader_.Accelerometer_Z};
    bs_tmp.quat = {imu_reader_.qw, imu_reader_.qx, imu_reader_.qy, imu_reader_.qz};
    base_state_buffer_.SetData(bs_tmp);
}

void RL_Opus::JointStateReadWriter()
{
    motor_ctrl_.Run(SetMotorCmd());
    RecordMotorState(motor_ctrl_.GetData());
}

unitree_hg::msg::dds_::LowCmd_ RL_Opus::SetMotorCmd()
{
    unitree_hg::msg::dds_::LowCmd_ cmd;
    auto mc = motor_command_buffer_.GetData();
    if (!mc) return cmd;
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        cmd.motor_cmd().at(i).q()   = mc->q_target[i];
        cmd.motor_cmd().at(i).dq()  = mc->dq_target[i];
        cmd.motor_cmd().at(i).kp()  = mc->kp[i];
        cmd.motor_cmd().at(i).kd()  = mc->kd[i];
        cmd.motor_cmd().at(i).tau() = mc->tau_ff[i];
    }
    return cmd;
}

void RL_Opus::RecordMotorState(const std::array<MotorData, 10> &data)
{
    MotorState ms_tmp;
    for (int i = 0; i < 10; ++i)
    {
        ms_tmp.q[i]       = data[i].q;
        ms_tmp.dq[i]      = data[i].dq;
        ms_tmp.tau_est[i] = 0.f;
    }
    motor_state_buffer_.SetData(ms_tmp);
}

volatile sig_atomic_t g_running = 1;
void sigint_handler(int) { g_running = 0; }

int main()
{
    signal(SIGINT, sigint_handler);
    RL_Opus rl_opus;
    while (g_running) { sleep(1); }
    std::cout << "\nSIGINT received, exiting..." << std::endl;
    return 0;
}
