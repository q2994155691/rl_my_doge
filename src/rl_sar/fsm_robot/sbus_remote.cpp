#include "sbus_remote.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

#ifdef __linux__
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#endif

namespace opus_remote
{
namespace
{

enum class SwitchPosition
{
    Up = 0,
    Mid,
    Down
};

struct DecodedSbus
{
    std::array<int, 16> channels{};
    bool lost_frame = false;
    bool failsafe = false;
};

float ScaleStick(int raw, int lo, int center, int hi, int deadband, bool invert)
{
    float value = 0.0f;
    if (raw >= center - deadband && raw <= center + deadband)
    {
        value = 0.0f;
    }
    else if (raw < center - deadband)
    {
        value = static_cast<float>(raw - (center - deadband)) /
                static_cast<float>((center - deadband) - lo);
    }
    else
    {
        value = static_cast<float>(raw - (center + deadband)) /
                static_cast<float>(hi - (center + deadband));
    }

    value = std::max(-1.0f, std::min(1.0f, value));
    return invert ? -value : value;
}

SwitchPosition GetSwitchPosition(int raw, int up_threshold, int down_threshold)
{
    if (raw < up_threshold) return SwitchPosition::Up;
    if (raw > down_threshold) return SwitchPosition::Down;
    return SwitchPosition::Mid;
}

Command SelectCommand(SwitchPosition ch5, SwitchPosition ch6)
{
    if (ch6 == SwitchPosition::Down) return Command::Passive; // raw high, about 1800
    if (ch6 == SwitchPosition::Up) return Command::GetDown;   // raw low, about 200
    if (ch5 == SwitchPosition::Up) return Command::StandUp;
    if (ch5 == SwitchPosition::Down) return Command::RLMode;
    return Command::None;
}

class SbusRemote
{
public:
    ~SbusRemote()
    {
        Close();
    }

    Command Update(RL &rl)
    {
        if (!Enabled(rl))
        {
            return Command::None;
        }

        const std::string port = rl.params.Get<std::string>("sbus_remote_port", "/dev/ttyUSB2");
        if (port != port_)
        {
            Close();
            port_ = port;
        }

        if (fd_ < 0 && !Open(port_))
        {
            ZeroCommand(rl);
            return Command::Passive;
        }

        ReadAvailable();

        DecodedSbus decoded;
        if (PopLatestFrame(decoded))
        {
            last_frame_time_ = std::chrono::steady_clock::now();
            if (decoded.failsafe || decoded.lost_frame)
            {
                ZeroCommand(rl);
                return Command::Passive;
            }

            ApplyAxes(rl, decoded);
            const int ch5_index = rl.params.Get<int>("sbus_remote_ch5_channel", 5) - 1;
            const int ch6_index = rl.params.Get<int>("sbus_remote_ch6_channel", 6) - 1;
            const int up_threshold = rl.params.Get<int>("sbus_remote_up_threshold", 600);
            const int down_threshold = rl.params.Get<int>("sbus_remote_down_threshold", 1400);

            if (!ValidChannel(ch5_index) || !ValidChannel(ch6_index))
            {
                return Command::None;
            }

            return SelectCommand(
                GetSwitchPosition(decoded.channels[ch5_index], up_threshold, down_threshold),
                GetSwitchPosition(decoded.channels[ch6_index], up_threshold, down_threshold));
        }

        const auto now = std::chrono::steady_clock::now();
        const double timeout = rl.params.Get<float>("sbus_remote_timeout", 0.25f);
        if (last_frame_time_.time_since_epoch().count() == 0 ||
            std::chrono::duration<double>(now - last_frame_time_).count() > timeout)
        {
            ZeroCommand(rl);
            return Command::Passive;
        }

        return Command::None;
    }

private:
    static constexpr int kFrameLen = 25;
    static constexpr uint8_t kStartByte = 0x0F;

    int fd_ = -1;
    std::string port_;
    std::vector<uint8_t> buffer_;
    std::chrono::steady_clock::time_point last_frame_time_{};

    bool Open(const std::string &port)
    {
#ifndef __linux__
        std::cout << LOGGER::ERROR << "SBUS remote is only implemented for Linux serial ports." << std::endl;
        return false;
#else
        fd_ = open(port.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0)
        {
            std::cout << LOGGER::ERROR << "Failed to open SBUS remote port " << port << ": "
                      << std::strerror(errno) << std::endl;
            return false;
        }

        termios2 tio{};
        if (ioctl(fd_, TCGETS2, &tio) != 0)
        {
            std::cout << LOGGER::ERROR << "Failed to read SBUS serial config: "
                      << std::strerror(errno) << std::endl;
            Close();
            return false;
        }

        tio.c_cflag &= ~CBAUD;
        tio.c_cflag |= BOTHER;
        tio.c_ispeed = 100000;
        tio.c_ospeed = 100000;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8 | CLOCAL | CREAD | PARENB | CSTOPB;
        tio.c_cflag &= ~PARODD;
        tio.c_iflag = IGNPAR;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;

        if (ioctl(fd_, TCSETS2, &tio) != 0)
        {
            std::cout << LOGGER::ERROR << "Failed to configure SBUS serial port: "
                      << std::strerror(errno) << std::endl;
            Close();
            return false;
        }

        std::cout << LOGGER::INFO << "SBUS remote enabled on " << port << " at 100000 8E2" << std::endl;
        return true;
#endif
    }

    void Close()
    {
        if (fd_ >= 0)
        {
            close(fd_);
            fd_ = -1;
        }
        buffer_.clear();
    }

    void ReadAvailable()
    {
        if (fd_ < 0) return;

        uint8_t data[256];
        while (true)
        {
            const ssize_t n = read(fd_, data, sizeof(data));
            if (n > 0)
            {
                buffer_.insert(buffer_.end(), data, data + n);
            }
            else
            {
                break;
            }
        }
    }

    bool PopLatestFrame(DecodedSbus &decoded)
    {
        bool found = false;
        DecodedSbus latest;

        size_t i = 0;
        while (i + kFrameLen <= buffer_.size())
        {
            if (buffer_[i] == kStartByte)
            {
                DecodedSbus candidate;
                if (DecodeFrame(&buffer_[i], candidate))
                {
                    latest = candidate;
                    found = true;
                }
                i += kFrameLen;
            }
            else
            {
                ++i;
            }
        }

        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<long>(i));
        if (found)
        {
            decoded = latest;
        }
        return found;
    }

    bool DecodeFrame(const uint8_t *d, DecodedSbus &out)
    {
        if (d[0] != kStartByte) return false;

        out.channels[0] = ((d[1] | d[2] << 8) & 0x07FF);
        out.channels[1] = ((d[2] >> 3 | d[3] << 5) & 0x07FF);
        out.channels[2] = ((d[3] >> 6 | d[4] << 2 | d[5] << 10) & 0x07FF);
        out.channels[3] = ((d[5] >> 1 | d[6] << 7) & 0x07FF);
        out.channels[4] = ((d[6] >> 4 | d[7] << 4) & 0x07FF);
        out.channels[5] = ((d[7] >> 7 | d[8] << 1 | d[9] << 9) & 0x07FF);
        out.channels[6] = ((d[9] >> 2 | d[10] << 6) & 0x07FF);
        out.channels[7] = ((d[10] >> 5 | d[11] << 3) & 0x07FF);
        out.channels[8] = ((d[12] | d[13] << 8) & 0x07FF);
        out.channels[9] = ((d[13] >> 3 | d[14] << 5) & 0x07FF);
        out.channels[10] = ((d[14] >> 6 | d[15] << 2 | d[16] << 10) & 0x07FF);
        out.channels[11] = ((d[16] >> 1 | d[17] << 7) & 0x07FF);
        out.channels[12] = ((d[17] >> 4 | d[18] << 4) & 0x07FF);
        out.channels[13] = ((d[18] >> 7 | d[19] << 1 | d[20] << 9) & 0x07FF);
        out.channels[14] = ((d[20] >> 2 | d[21] << 6) & 0x07FF);
        out.channels[15] = ((d[21] >> 5 | d[22] << 3) & 0x07FF);

        const uint8_t flags = d[23];
        out.lost_frame = (flags & 0x04) != 0;
        out.failsafe = (flags & 0x08) != 0;
        return true;
    }

    void ApplyAxes(RL &rl, const DecodedSbus &decoded)
    {
        const int lo = rl.params.Get<int>("sbus_remote_raw_min", 200);
        const int center = rl.params.Get<int>("sbus_remote_raw_center", 1000);
        const int hi = rl.params.Get<int>("sbus_remote_raw_max", 1800);
        const int deadband = rl.params.Get<int>("sbus_remote_deadband", 20);

        const int x_index = rl.params.Get<int>("sbus_remote_x_channel", 2) - 1;
        const int y_index = rl.params.Get<int>("sbus_remote_y_channel", 4) - 1;
        const int yaw_index = rl.params.Get<int>("sbus_remote_yaw_channel", 1) - 1;

        if (ValidChannel(x_index))
        {
            rl.control.x = ScaleStick(decoded.channels[x_index], lo, center, hi, deadband,
                                      rl.params.Get<bool>("sbus_remote_invert_x", false));
        }
        if (ValidChannel(y_index))
        {
            rl.control.y = ScaleStick(decoded.channels[y_index], lo, center, hi, deadband,
                                      rl.params.Get<bool>("sbus_remote_invert_y", false));
        }
        if (ValidChannel(yaw_index))
        {
            rl.control.yaw = ScaleStick(decoded.channels[yaw_index], lo, center, hi, deadband,
                                        rl.params.Get<bool>("sbus_remote_invert_yaw", false));
        }
    }

    static bool ValidChannel(int index)
    {
        return index >= 0 && index < 16;
    }

    static void ZeroCommand(RL &rl)
    {
        rl.control.x = 0.0f;
        rl.control.y = 0.0f;
        rl.control.yaw = 0.0f;
    }
};

SbusRemote &Remote()
{
    static SbusRemote remote;
    return remote;
}

} // namespace

bool Enabled(RL &rl)
{
    return rl.params.Get<bool>("sbus_remote_enabled", false);
}

Command Update(RL &rl)
{
    return Remote().Update(rl);
}

std::string StateFromRemoteCommand(Command command, const std::string &current_state)
{
    switch (command)
    {
    case Command::Passive:
        return "RLFSMStatePassive";
    case Command::StandUp:
        return "RLFSMStateGetUp";
    case Command::RLMode:
        return "RLFSMStateRLLocomotion";
    case Command::GetDown:
        return "RLFSMStateGetDown";
    case Command::None:
    default:
        return current_state;
    }
}

} // namespace opus_remote
