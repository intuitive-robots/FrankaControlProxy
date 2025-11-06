#ifndef MODE_ID_HPP
#define MODE_ID_HPP
#include <cstdint>
#include <string>

namespace protocol {

#include <cstdint>
#include <string>

enum class ModeID : uint8_t {
    IDLE               = 0x00,  // Robot idle / no control command
    JOINT_POSITION     = 0x01,  // Position control in joint space
    JOINT_VELOCITY     = 0x02,  // Velocity control in joint space
    CARTESIAN_POSE     = 0x03,  // Pose control in Cartesian space
    CARTESIAN_VELOCITY = 0x04,  // Velocity control in Cartesian space
    JOINT_TORQUE       = 0x05,  // Joint torque control mode
    GRAVITY_COMP       = 0x06   // Gravity compensation mode
};

inline std::string toString(ModeID mode) {
    switch (mode) {
        case ModeID::IDLE:               return "IDLE";
        case ModeID::JOINT_POSITION:     return "JOINT_POSITION";
        case ModeID::JOINT_VELOCITY:     return "JOINT_VELOCITY";
        case ModeID::CARTESIAN_POSE:     return "CARTESIAN_POSE";
        case ModeID::CARTESIAN_VELOCITY: return "CARTESIAN_VELOCITY";
        case ModeID::JOINT_TORQUE:       return "JOINT_TORQUE";
        case ModeID::GRAVITY_COMP:       return "GRAVITY_COMP";
        default:                         return "UNKNOWN_MODE";
    }
}

// enum class ModeID : uint8_t {
//     CARTESIAN_POSITION = 0,
//     CARTESIAN_VELOCITY = 1,
//     JOINT_POSITION = 2,
//     JOINT_VELOCITY = 3,
//     HUMAN_MODE = 4,
//     IDLE = 5,
//     PD_TEST = 6,
// };

// inline std::string toString(ModeID mode) {
//     switch (mode) {
//         case ModeID::IDLE: return "idle";
//         case ModeID::HUMAN_MODE: return "zero_torque";
//         case ModeID::PD_TEST: return "joint_pd";
//         default: return "idle";
//     }
// }

}  // namespace protocol
#endif // MODE_ID_HPP    