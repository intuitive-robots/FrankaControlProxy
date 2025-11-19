#pragma once
#include <vector>
#include <cstdint>

namespace protocol {
struct GraspCommand {
    double width;      // Desired width of the gripper in meters
    double speed;      // Speed at which to close the gripper in meters per second
    double force;      // Force to apply when closing the gripper in Newtons

    GraspCommand() : width(0.0f), speed(0.0f), force(0.0f) {}
    GraspCommand(float w, float s, float f) : width(w), speed(s), force(f) {}
};
}