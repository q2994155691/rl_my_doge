#include <iostream>
#include <csignal>
#include <unistd.h>
#include <chrono>

#include "unitree/common/thread/recurrent_thread.hpp"
#include "user/Motor_thread.hpp"
#include "user/IMUReader.h"
#include "unitree/g1/motors.hpp"
#include "unitree/g1/data_buffer.hpp"
#include "unitree/g1/base_state.hpp"

using namespace unitree::common;

volatile sig_atomic_t g_running = 1;
void sigint_handler(int) { g_running = 0; }

// --- globals ---
// MotorController Motor_control;
// DataBuffer<MotorCommand> motor_command_buffer_;
// DataBuffer<MotorState>   motor_state_buffer_;
IMUReader imuReader;
DataBuffer<BaseState>    base_state_buffer_;
// int tick = 0;

// unitree_hg::msg::dds_::LowCmd_ SetMotorCmd() {
//     unitree_hg::msg::dds_::LowCmd_ cmd;
//     auto mc = motor_command_buffer_.GetData();
//     if (!mc) return cmd;
//     for (int i = 0; i < G1_NUM_MOTOR; i++) {
//         cmd.motor_cmd().at(i).q()   = mc->q_target[i];
//         cmd.motor_cmd().at(i).dq()  = mc->dq_target[i];
//         cmd.motor_cmd().at(i).kp()  = mc->kp[i];
//         cmd.motor_cmd().at(i).kd()  = mc->kd[i];
//         cmd.motor_cmd().at(i).tau() = mc->tau_ff[i];
//     }
//     return cmd;
// }

// void RecordMotorState(const std::array<MotorData, 10>& data) {
//     MotorState ms;
//     for (int i = 0; i < G1_NUM_MOTOR; i++) {
//         ms.q[i]  = data[i].q;
//         ms.dq[i] = data[i].dq;
//     }
//     motor_state_buffer_.SetData(ms);
// }

void RecordBaseState() {
    BaseState bs_tmp;
    bs_tmp.omega = {imuReader.RollSpeed, imuReader.PitchSpeed, imuReader.HeadingSpeed};
    bs_tmp.rpy = {imuReader.Roll, imuReader.Pitch, imuReader.Heading};
    bs_tmp.acc = {imuReader.Accelerometer_X, imuReader.Accelerometer_Y, imuReader.Accelerometer_Z};
    bs_tmp.quat = {imuReader.qw, imuReader.qx, imuReader.qy, imuReader.qz};
    base_state_buffer_.SetData(bs_tmp);
}

void IMUStateReader() {
    static int imu_tick = 0;
    static auto last_time = std::chrono::steady_clock::now();

    if (imuReader.fetchIMUData()) {
        RecordBaseState();
        ++imu_tick;
        if (imu_tick % 200 == 0) {
            auto now = std::chrono::steady_clock::now();
            double elapsed_ms = std::chrono::duration<double, std::milli>(now - last_time).count();
            double actual_hz = 200.0 / (elapsed_ms / 1000.0);
            std::cout << "[IMU] tick=" << imu_tick
                      << " elapsed=" << elapsed_ms << "ms"
                      << " actual_freq=" << actual_hz << "Hz" << std::endl;
            last_time = now;
        }
    } else {
        std::cerr << "Failed to fetch IMU data" << std::endl;
    }
}

// void JointStateReadWriter() {
//     Motor_control.Run(SetMotorCmd());
//     RecordMotorState(Motor_control.GetData());
//
//     if (++tick % 500 == 0) {
//         std::cout << "[tick " << tick << "] fb_q=" << Motor_control.GetData()[0].q << " cmd_q=" << Motor_control.current_cmd_.motor_cmd().at(0).q() << " kp=" << Motor_control.current_cmd_.motor_cmd().at(0).kp() << " kd=" << Motor_control.current_cmd_.motor_cmd().at(0).kd() << std::endl;
//     }
// }

int main() {
    signal(SIGINT, sigint_handler);
    // // 電機線程暫時關閉
    // MotorCommand cmd{};
    // for (int i = 0; i < G1_NUM_MOTOR; i++) {
    //     cmd.kp[i] = 1.0f;
    //     cmd.kd[i] = 0.05f;
    // }
    // motor_command_buffer_.SetData(cmd);
    // float motor_dt_ = 0.002;
    // ThreadPtr jsrw_thread = CreateRecurrentThreadEx(
    //     "command_writer", UT_CPU_ID_NONE, motor_dt_ * 1e6,
    //     &JointStateReadWriter);
    // usleep(0.1 * 1e6);

    // 200Hz = 5000us IMU 線程 (匹配 AHRS 輸出頻率)
    PyEval_ReleaseLock();
    ThreadPtr imu_thread = CreateRecurrentThreadEx(
        "imu", UT_CPU_ID_NONE, 0.005 * 1e6,
        &IMUStateReader);

    usleep(0.1 * 1e6);

    std::cout << "JointStateReadWriter running at 500Hz, IMUStateReader at 200Hz. Ctrl+C to stop." << std::endl;

    // 主線程掛起
    while (g_running) {
        sleep(1);
    }
    std::cout << "\nSIGINT received, exiting..." << std::endl;
    return 0;
}
