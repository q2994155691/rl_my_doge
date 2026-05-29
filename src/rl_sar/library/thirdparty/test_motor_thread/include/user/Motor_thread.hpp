#include <unistd.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <array>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <csignal>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <fstream>
#include <iomanip>
#include <unitree/idl/hg/LowCmd_.hpp>

struct SerialGroup {
    const char *port;
    std::vector<int> motorIDs;
};

class MotorController {
public:
    // A1-like layout: port N = leg N, motors [3N, 3N+1, 3N+2] = [hip, thigh, calf]
    std::vector<SerialGroup> serialGroups = {
        {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC04K70-if03-port0", {0, 1, 2}},   // FR leg: hip + thigh + calf
        {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC04K70-if02-port0", {3, 4, 5}},   // FL leg: hip + thigh + calf
        {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC04K70-if01-port0", {6, 7, 8}},   // RR leg: hip + thigh + calf
        {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC04K70-if00-port0", {9, 10, 11}}, // RL leg: hip + thigh + calf
    };
    MotorController() {
        InitializeSerialPorts();
        for(std::array<ThreadData, 5>::iterator td = threadData.begin(); td != threadData.end(); ++td) {
            td->start_time = std::chrono::high_resolution_clock::now();
        }
        // Start the motor control thread
        workerThreads[0] = std::thread(&MotorController::RunThread<0>, this);
        workerThreads[1] = std::thread(&MotorController::RunThread<1>, this);
        workerThreads[2] = std::thread(&MotorController::RunThread<2>, this);
        workerThreads[3] = std::thread(&MotorController::RunThread<3>, this);
        workerThreads[4] = std::thread(&MotorController::MonitorThread, this);
        std::cout << "Start motor thread： Done!" << std::endl;
    }
    ~MotorController() {
        Stop();
        if (dataFile.is_open()) {
            dataFile.close();
        }
    }
    struct ThreadData {
        std::atomic<int> count{0};
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    };

    std::array<ThreadData, 5> threadData;
    std::mutex printMutex;
    std::atomic<bool> running{true};
    std::array<std::mutex, 12> motorMutexes;
    std::mutex fileMutex;
    std::array<std::thread, 5> workerThreads;
    std::array<float, 12> rawMotorQ = {};

    unitree_hg::msg::dds_::LowCmd_ current_cmd_;
    std::mutex cmd_mutex_;

    void Run(const unitree_hg::msg::dds_::LowCmd_& dds_low_command) {
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            current_cmd_ = dds_low_command;
        }

    }

    void Stop() {
        running = false;
        for(std::thread& thread : workerThreads) {
            if(thread.joinable()) {
                thread.join();
            }
        }
        std::cout << "All worker threads stopped" << std::endl;
    }

public:
    /// Startq（0位偏移）： 左腿roll 内扣，则需增大，右腿内扣则需减小
    // 趴姿校準（上電時姿態）：Startq[i] = -pre_running_pos[i]
    // pre_running_pos = {0.00, 1.36, -2.65} per leg (hip, thigh, calf)
    // Layout: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
    std::array<float, 12> Startq = { 1.011058, 1.281201, -2.444139,   // FR: hip, thigh, calf
                                    -0.189573, -0.807788, 2.900948,   // FL: hip, thigh, calf
                                    -0.315956, 1.737757, -2.541043,   // RR: hip, thigh, calf
                                     1.484992, -0.372828, 2.905213 }; // RL: hip, thigh, calf

    // Per-motor direction sign (+1 or -1). Right-side legs (FR, RR) are physically
    // mounted mirrored to left-side legs (FL, RL); the logical q convention is
    // "q 增大 = 向站立方向" for left, so right-side motors need -1 to flip.
    // Layout: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
    std::array<int, 12> Sign =    {  1,  -1,  -1,   // FR: hip, thigh, calf
                                    +1,  +1,  +1,   // FL: hip, thigh, calf
                                     1,  -1,  -1,   // RR: hip, thigh, calf (same as FR)
                                    1,  +1,  +1 }; // RL: hip, thigh, calf (same as FL)

    std::array<MotorData, 12> allMotorData;
    float Speed_Ratio = 6.33;
    float Gear_Ratio = 3.;
    std::vector<std::unique_ptr<SerialPort>> serialPorts;

    std::ofstream dataFile;
    std::chrono::time_point<std::chrono::system_clock> lastSaveTime;
    const std::chrono::milliseconds saveInterval{4}; // 100ms保存一次

    void InitializeSerialPorts() {
        for(std::vector<SerialGroup>::iterator group = serialGroups.begin(); group != serialGroups.end(); ++group) {
            std::unique_ptr<SerialPort> port = std::make_unique<SerialPort>(group->port);
            serialPorts.push_back(std::move(port));
        }
    }

    template<int N>
    void RunThread() {

        SerialPort& serial = *serialPorts[N];
        ThreadData& td = threadData[N];
        
         while(running)
           {
            std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();
            
            for(std::vector<int>::iterator motorID = serialGroups[N].motorIDs.begin(); 
                motorID != serialGroups[N].motorIDs.end(); ++motorID) {
                MotorCmd cmd;
                MotorData data;
                
                ConfigureMotorCommand(cmd, *motorID, current_cmd_);
                data.motorType = MotorType::GO_M8010_6;
                serial.sendRecv(&cmd, &data);
                ParseMotorFeedback(data, *motorID);
            }
            td.count++;
          }
    }

    template<int N>
    void PrintMotorQ() {
        static auto last_raw_log = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_raw_log).count() < 1.0) {
            return;
        }
        last_raw_log = now;

        std::cout << "[Motor raw q]";
        for (int motorID : serialGroups[N].motorIDs) {
            std::cout << " id=" << motorID
                      << " raw_q=" << std::fixed << std::setprecision(4) << rawMotorQ[motorID];
        }
        std::cout << " [Motor logic q]";
        for (int motorID : serialGroups[N].motorIDs) {
            std::cout << " id=" << motorID
                      << " logic_q=" << std::fixed << std::setprecision(6) << allMotorData[motorID].q;
        }
        std::cout << std::endl;
    }

    void MonitorThread() {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            std::lock_guard<std::mutex> lock(printMutex);
            std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
            
            for(int i = 0; i < serialGroups.size(); ++i) {
                long elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    now - threadData[i].start_time).count();
                int freq = elapsed > 0 ? threadData[i].count / elapsed : 0;
                
                threadData[i].count = 0;
                threadData[i].start_time = now;
            }
    }

    int CalculateChannelID(int motorID) {
        return motorID % 3;
    }

    // A1 layout: all motors same type, no special gear ratio
    bool IsSpecialMotor(int motorID) const {
        (void)motorID;
        return false;
    }

    void PrintFeedback() {
        for (int i = 0; i < 12; ++i) {
            std::cout << "m" << i << ": " << allMotorData[i].q;
        }
        std::cout << std::endl;
    }

    void ConfigureMotorCommand(MotorCmd& cmd, int motorID, const unitree_hg::msg::dds_::LowCmd_& dds_low_command) {
        cmd.motorType = MotorType::GO_M8010_6;
        cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        cmd.id = CalculateChannelID(motorID);
        cmd.kp = dds_low_command.motor_cmd().at(motorID).kp();
        cmd.kd = dds_low_command.motor_cmd().at(motorID).kd();
        cmd.tau = dds_low_command.motor_cmd().at(motorID).tau();
        
        const bool is_special = IsSpecialMotor(motorID);
        const float ratio = is_special ? (Speed_Ratio * Gear_Ratio) : Speed_Ratio;
        
        const int sign = Sign[motorID];
        cmd.q = (dds_low_command.motor_cmd().at(motorID).q() * sign + Startq[motorID]) * ratio;
        cmd.dq = dds_low_command.motor_cmd().at(motorID).dq() * sign * ratio;
    }

    void ParseMotorFeedback(MotorData& data, int motorID) {
        const bool is_special = IsSpecialMotor(motorID);
        const float ratio = is_special ? (Speed_Ratio * Gear_Ratio) : Speed_Ratio;
        const int sign = Sign[motorID];

        rawMotorQ.at(motorID) = data.q;
        allMotorData.at(motorID).q = (data.q / ratio - Startq[motorID]) * sign;
        allMotorData.at(motorID).dq = data.dq / ratio * sign;
    }

    const std::array<MotorData, 12> &GetData() const {
        return allMotorData;
    }
};
