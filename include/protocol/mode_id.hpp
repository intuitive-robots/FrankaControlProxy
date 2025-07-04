#ifndef MODE_ID_HPP
#define MODE_ID_HPP
#include <cstdint>
#include <string>

namespace protocol {

enum class ModeID : uint8_t {
    CARTESIAN_POSITION = 0,
    CARTESIAN_VELOCITY = 1,
    JOINT_POSITION = 2,
    JOINT_VELOCITY = 3,
    HUMAN_MODE = 4,
    IDLE = 5,
};

inline std::string toString(ModeID mode) {
    switch (mode) {
        case ModeID::IDLE: return "idle";
        case ModeID::JOINT_VELOCITY: return "joint_velocity";
        default: return "unknown";
    }
}

}  // namespace protocol
#endif // MODE_ID_HPP    