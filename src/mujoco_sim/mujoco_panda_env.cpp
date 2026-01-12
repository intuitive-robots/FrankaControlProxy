#include "mujoco_sim/mujoco_panda_env.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <utility>
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
    const auto sim_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(data_->time));
    robot_state.time = franka::Duration(static_cast<uint64_t>(sim_ns.count()));
}

void MujocoPandaEnv::refreshRobotState(franka::RobotState& robot_state)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    _refreshRobotState(robot_state);
}
