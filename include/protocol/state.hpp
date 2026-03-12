#pragma once

#include <franka/robot_state.h>
#include <msgpack.hpp>
#include <vector>

#include "utils/Pose.h"

struct FrankaArmState
{
    uint32_t time_ms;
    std::vector<double> EE_pos;
    std::vector<double> EE_quat;
    std::vector<double> O_T_EE;
    std::vector<double> O_T_EE_d;
    std::vector<double> q;
    std::vector<double> q_d;
    std::vector<double> dq;
    std::vector<double> dq_d;
    std::vector<double> tau_ext_hat_filtered;
    std::vector<double> O_F_ext_hat_K;
    std::vector<double> K_F_ext_hat_K;
    MSGPACK_DEFINE_MAP(time_ms, EE_pos, EE_quat, O_T_EE, O_T_EE_d, q, q_d, dq, dq_d, tau_ext_hat_filtered,
                       O_F_ext_hat_K, K_F_ext_hat_K)

    FrankaArmState(const franka::RobotState& state)
        : time_ms(0),
          O_T_EE(state.O_T_EE.begin(), state.O_T_EE.end()),
          O_T_EE_d(state.O_T_EE_d.begin(), state.O_T_EE_d.end()),
          q(state.q.begin(), state.q.end()),
          q_d(state.q_d.begin(), state.q_d.end()),
          dq(state.dq.begin(), state.dq.end()),
          dq_d(state.dq_d.begin(), state.dq_d.end()),
          tau_ext_hat_filtered(state.tau_ext_hat_filtered.begin(),
                               state.tau_ext_hat_filtered.end()),
          O_F_ext_hat_K(state.O_F_ext_hat_K.begin(), state.O_F_ext_hat_K.end()),
          K_F_ext_hat_K(state.K_F_ext_hat_K.begin(), state.K_F_ext_hat_K.end())
          {
            transform::Pose pose_EE(state.O_T_EE);
            Eigen::Vector3d t = pose_EE.translation();
            EE_pos = {t.x(), t.y(), t.z()};
            Eigen::Quaterniond q = pose_EE.quaternion();
            EE_quat = {q.x(), q.y(), q.z(), q.w()};
          };
};

struct PandaGripperState
{
    double width;
    double max_width;
    bool is_grasped;
    uint16_t temperature;
    MSGPACK_DEFINE_MAP(width, max_width, is_grasped, temperature)
};

struct FrankaResponseMsg
{
    std::string code;
    std::vector<uint8_t> payload;
    MSGPACK_DEFINE_MAP(code, payload)
};