#pragma once
#include <vector>
#include <cstdint>


struct GraspCommand {
    float width;      // Desired width of the gripper in meters
    float speed;      // Speed at which to close the gripper in meters per second
    float force;      // Force to apply when closing the gripper in Newtons

    GraspCommand() : width(0.0f), speed(0.0f), force(0.0f) {}
    GraspCommand(std::vector<uint8_t>::const_iterator begin) {
        width = *reinterpret_cast<const float*>(&(*begin));
        speed = *reinterpret_cast<const float*>(&(*(begin + sizeof(float))));
        force = *reinterpret_cast<const float*>(&(*(begin + 2 * sizeof(float))));
    }
};