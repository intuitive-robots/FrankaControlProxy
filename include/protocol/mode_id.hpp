#pragma once
#include <cstdint>
#include <string>
#include <stdexcept>

namespace protocol {

enum class ModeID : uint8_t {
    IDLE               = 0x00,  // Robot idle / no control command
    JOINT_POSITION     = 0x01,  // Position control in joint space
    JOINT_VELOCITY     = 0x02,  // Velocity control in joint space
    CARTESIAN_POSE     = 0x03,  // Pose control in Cartesian space
    CARTESIAN_VELOCITY = 0x04,  // Velocity control in Cartesian space
    JOINT_TORQUE       = 0x05,  // Joint torque control mode
    HUMAN_CONTROL       = 0x06   // human control mode
};

struct FrankaArmControlMode {
    ModeID id;
    std::string url; 
};

inline std::string toString(const ModeID mode) {
    switch (mode) {
        case ModeID::IDLE:               return "IDLE";
        case ModeID::JOINT_POSITION:     return "JOINT_POSITION";
        case ModeID::JOINT_VELOCITY:     return "JOINT_VELOCITY";
        case ModeID::CARTESIAN_POSE:     return "CARTESIAN_POSE";
        case ModeID::CARTESIAN_VELOCITY: return "CARTESIAN_VELOCITY";
        case ModeID::JOINT_TORQUE:       return "JOINT_TORQUE";
        case ModeID::HUMAN_CONTROL:       return "HUMAN_CONTROL";
        default:                         return "UNKNOWN_MODE";
    }
}

inline ModeID fromString(const std::string& mode_str) {
    if (mode_str == "IDLE")               return ModeID::IDLE;
    if (mode_str == "JOINT_POSITION")     return ModeID::JOINT_POSITION;
    if (mode_str == "JOINT_VELOCITY")     return ModeID::JOINT_VELOCITY;
    if (mode_str == "CARTESIAN_POSE")     return ModeID::CARTESIAN_POSE;
    if (mode_str == "CARTESIAN_VELOCITY") return ModeID::CARTESIAN_VELOCITY;
    if (mode_str == "JOINT_TORQUE")       return ModeID::JOINT_TORQUE;
    if (mode_str == "HUMAN_CONTROL")       return ModeID::HUMAN_CONTROL;
    throw std::invalid_argument("Unknown ModeID string: " + mode_str);
}

}  // namespace protocol