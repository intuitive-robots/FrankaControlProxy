#pragma once
#include <array>
#include <msgpack.hpp>

struct JointCommand
{
    std::array<double, 7> pos;
    std::array<double, 7> vel;
    MSGPACK_DEFINE_MAP(pos, vel)
};

struct CartesianPoseCommand
{
    std::array<double, 3> pos;
    std::array<double, 4> rot;
    std::array<double, 3> pos_vel;
    std::array<double, 4> rot_vel;
    MSGPACK_DEFINE_MAP(pos, rot, pos_vel, rot_vel)
};

struct GraspCommandMsg
{
    double width;
    double speed;
    double force;
    MSGPACK_DEFINE_MAP(width, speed, force)
};