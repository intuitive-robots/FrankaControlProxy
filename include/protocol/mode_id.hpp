#pragma once
#include <string>
#include <cstdint>
#include <string>
#include <stdexcept>

enum class ControlModeID : std::string {
    IDLE               = "",  // Robot idle / no control command
    JOINT_POSITION     = 0x01,  // Position control in joint space
    JOINT_VELOCITY     = 0x02,  // Velocity control in joint space
    CARTESIAN_POSE     = 0x03,  // Pose control in Cartesian space
    CARTESIAN_VELOCITY = 0x04,  // Velocity control in Cartesian space
    JOINT_TORQUE       = 0x05,  // Joint torque control mode
    HUMAN_CONTROL       = 0x06   // human control mode
};


struct FrankaArmControlMode {
    ControlModeID id;
    std::string url; 
};

inline std::string toString(const ControlModeID mode) {
    switch (mode) {
        case ControlModeID::IDLE:               return "IDLE";
        case ControlModeID::JOINT_POSITION:     return "JOINT_POSITION";
        case ControlModeID::JOINT_VELOCITY:     return "JOINT_VELOCITY";
        case ControlModeID::CARTESIAN_POSE:     return "CARTESIAN_POSE";
        case ControlModeID::CARTESIAN_VELOCITY: return "CARTESIAN_VELOCITY";
        case ControlModeID::JOINT_TORQUE:       return "JOINT_TORQUE";
        case ControlModeID::HUMAN_CONTROL:       return "HUMAN_CONTROL";
        default:                         return "UNKNOWN_MODE";
    }
}

inline ControlModeID fromString(const std::string& mode_str) {
    if (mode_str == "IDLE")               return ControlModeID::IDLE;
    if (mode_str == "JOINT_POSITION")     return ControlModeID::JOINT_POSITION;
    if (mode_str == "JOINT_VELOCITY")     return ControlModeID::JOINT_VELOCITY;
    if (mode_str == "CARTESIAN_POSE")     return ControlModeID::CARTESIAN_POSE;
    if (mode_str == "CARTESIAN_VELOCITY") return ControlModeID::CARTESIAN_VELOCITY;
    if (mode_str == "JOINT_TORQUE")       return ControlModeID::JOINT_TORQUE;
    if (mode_str == "HUMAN_CONTROL")       return ControlModeID::HUMAN_CONTROL;
    throw std::invalid_argument("Unknown ControlModeID string: " + mode_str);
}