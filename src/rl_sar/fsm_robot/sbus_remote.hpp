#ifndef OPUS_SBUS_REMOTE_HPP
#define OPUS_SBUS_REMOTE_HPP

#include "rl_sdk.hpp"

#include <string>

namespace opus_remote
{

enum class Command
{
    None = 0,
    StandUp,
    RLMode,
    GetDown,
    Passive
};

bool Enabled(RL &rl);
Command Update(RL &rl);
std::string StateFromRemoteCommand(Command command, const std::string &current_state);

} // namespace opus_remote

#endif // OPUS_SBUS_REMOTE_HPP
