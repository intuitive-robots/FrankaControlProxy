#pragma once
#include <vector>
#include <cstdint>

namespace protocol {
struct GraspCommand {
    float width;      // Desired width of the gripper in meters
    float speed;      // Speed at which to close the gripper in meters per second
    float force;      // Force to apply when closing the gripper in Newtons

    GraspCommand() : width(0.0f), speed(0.0f), force(0.0f) {}
};
}