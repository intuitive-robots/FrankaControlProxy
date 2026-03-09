#include "mujoco_sim/mujoco_panda_env.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <utility>
#include <vector>

#include <zerolancom/zerolancom.hpp>

MujocoPandaEnv::MujocoPandaEnv(const std::string& model_path) : model_path_(model_path) {}

MujocoPandaEnv::~MujocoPandaEnv()
{
    stop();
}

void MujocoPandaEnv::start()
{
    if (initialized_)
    {
        throw std::runtime_error("MujocoPandaEnv is already started");
    }
    loadModel();
    initialized_ = true;
}

void MujocoPandaEnv::stop()
{
    // No specific stop actions needed for MuJoCo simulation.
}

void MujocoPandaEnv::updateStateSnapshot(mjData& snapShot) const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    mj_copyData(&snapShot, model_.get(), data_.get());
}

void MujocoPandaEnv::loadModel()
{
    char error[1024] = {0};
    model_.reset(mj_loadXML(model_path_.c_str(), nullptr, error, sizeof(error)));
    if (!model_)
    {
        std::cerr << "Could not load model: " << error << std::endl;
        throw std::runtime_error("Failed to load MuJoCo model");
    }
    data_.reset(mj_makeData(model_.get()));
    if (!data_)
    {
        std::cerr << "Could not allocate mjData" << std::endl;
        throw std::runtime_error("Failed to create MuJoCo data");
    }

    int key_id = mj_name2id(model_.get(), mjOBJ_KEY, "home");
    if (key_id != -1)
    {
        mj_resetDataKeyframe(model_.get(), data_.get(), key_id);
        mj_forward(model_.get(), data_.get());
    }
    else
    {
        zlc::warn("Keyframe 'home' not found in the model");
    }
}

void MujocoPandaEnv::nextStep(const franka::Torques& torques, franka::RobotState& robot_state)
{
    if (!initialized_ || !model_ || !data_)
    {
        return;
    }
    const int torque_count = std::min<int>(torques.tau_J.size(), model_->nu);
    // std::fill_n(data_->ctrl, model_->nu, 0.0);
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (int i = 0; i < torque_count; ++i)
    {
        data_->ctrl[i] = torques.tau_J[i] + data_->qfrc_bias[i];
    }
    mj_step(model_.get(), data_.get());
    _refreshRobotState(robot_state);
}

void MujocoPandaEnv::_refreshRobotState(franka::RobotState& robot_state)
{
    const int joint_count = std::min<int>(7, model_->nv);
    const int torque_count = std::min<int>(7, model_->nu);

    for (int i = 0; i < joint_count; ++i)
    {
        robot_state.q[i] = data_->qpos[i];
        robot_state.q_d[i] = data_->qpos[i];
        robot_state.dq[i] = data_->qvel[i];
        robot_state.dq_d[i] = data_->qvel[i];
    }
    for (int i = 0; i < torque_count; ++i)
    {
        robot_state.tau_J[i] = data_->ctrl[i];
        robot_state.tau_J_d[i] = data_->ctrl[i];
    }

    // Compute O_T_EE (end-effector pose in base frame)
    int hand_body_id = mj_name2id(model_.get(), mjOBJ_BODY, "hand");
    if (hand_body_id >= 0)
    {
        // Get position (xpos is body_id * 3)
        const double* pos = &data_->xpos[hand_body_id * 3];
        // Get rotation matrix (xmat is body_id * 9, row-major 3x3)
        const double* rot = &data_->xmat[hand_body_id * 9];

        // O_T_EE is column-major 4x4: [R11,R21,R31,0, R12,R22,R32,0, R13,R23,R33,0, tx,ty,tz,1]
        // MuJoCo xmat is row-major: [R11,R12,R13, R21,R22,R23, R31,R32,R33]
        robot_state.O_T_EE[0] = rot[0];  // R11
        robot_state.O_T_EE[1] = rot[3];  // R21
        robot_state.O_T_EE[2] = rot[6];  // R31
        robot_state.O_T_EE[3] = 0.0;
        robot_state.O_T_EE[4] = rot[1];  // R12
        robot_state.O_T_EE[5] = rot[4];  // R22
        robot_state.O_T_EE[6] = rot[7];  // R32
        robot_state.O_T_EE[7] = 0.0;
        robot_state.O_T_EE[8] = rot[2];  // R13
        robot_state.O_T_EE[9] = rot[5];  // R23
        robot_state.O_T_EE[10] = rot[8]; // R33
        robot_state.O_T_EE[11] = 0.0;
        robot_state.O_T_EE[12] = pos[0]; // tx
        robot_state.O_T_EE[13] = pos[1]; // ty
        robot_state.O_T_EE[14] = pos[2]; // tz
        robot_state.O_T_EE[15] = 1.0;

        // Compute Cartesian velocities using Jacobian
        std::vector<double> jacp(3 * model_->nv, 0.0);
        std::vector<double> jacr(3 * model_->nv, 0.0);
        mj_jacBody(model_.get(), data_.get(), jacp.data(), jacr.data(), hand_body_id);

        // Compute Cartesian velocity: v = J * dq
        // O_dP_EE_c and O_dP_EE_d are 6-element arrays: [vx, vy, vz, wx, wy, wz]
        for (int i = 0; i < 3; ++i)
        {
            double linear_vel = 0.0;
            double angular_vel = 0.0;
            for (int j = 0; j < joint_count; ++j)
            {
                // MuJoCo Jacobian is row-major: jacp[i * nv + j]
                linear_vel += jacp[i * model_->nv + j] * data_->qvel[j];
                angular_vel += jacr[i * model_->nv + j] * data_->qvel[j];
            }
            robot_state.O_dP_EE_c[i] = linear_vel;
            robot_state.O_dP_EE_c[i + 3] = angular_vel;
            robot_state.O_dP_EE_d[i] = linear_vel;
            robot_state.O_dP_EE_d[i + 3] = angular_vel;
        }
    }

    const auto sim_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(data_->time));
    robot_state.time = franka::Duration(static_cast<uint64_t>(sim_ns.count()));
}

void MujocoPandaEnv::refreshRobotState(franka::RobotState& robot_state)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    _refreshRobotState(robot_state);
}
