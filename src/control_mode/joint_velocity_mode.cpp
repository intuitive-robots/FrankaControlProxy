#include "joint_velocity_mode.hpp"
#include <franka/exception.h>
#include <franka/robot_state.h>
#include "utils/logger.hpp"
#include "protocol/codec.hpp"

JointVelocityMode::JointVelocityMode():
    desired_velocities_(franka::JointVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}})
{};
JointVelocityMode::~JointVelocityMode() = default;


void JointVelocityMode::controlLoop() {
    LOG_INFO("[JointVelocityMode] Started.");
    is_running_ = true;
    // Initialize desired velocities to zero
    desired_velocities_.write(franka::JointVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

    if (!robot_ || !model_) {
        LOG_ERROR("[JointVelocityMode] Robot or model not set.");
        return;
    }
    robot_->automaticErrorRecovery();

    std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)> joint_velocity_callback =
        [this](const franka::RobotState& state, franka::Duration) -> franka::JointVelocities {
            if (!is_running_) {
                throw franka::ControlException("JointVelocityMode stopped.");
            }
            updateRobotState(state);
            franka::JointVelocities desired_velocities = desired_velocities_.read();
            if (!is_running_) {
                return franka::MotionFinished(desired_velocities);
            }
            return desired_velocities;
        };

    try {
        robot_->control(joint_velocity_callback);
    } catch (const franka::ControlException& e) {
        LOG_ERROR("[JointVelocityMode] Exception: {}", e.what());
        if (std::string(e.what()).find("reflex") != std::string::npos) {
            LOG_WARN("[JointVelocityMode] Reflex detected, attempting automatic recovery...");
            try {
                robot_->automaticErrorRecovery();
            } catch (const franka::Exception& recovery_error) {
                LOG_ERROR("Recovery failed: {}", recovery_error.what());
            }
        }
        LOG_INFO("[JointVelocityMode] Exited.");
    }
}


protocol::ModeID JointVelocityMode::getModeID() const {
    return protocol::ModeID::JOINT_VELOCITY;
}

void JointVelocityMode::writeCommand(const protocol::ByteView& data) {
    franka::JointVelocities velocities = protocol::decode<franka::JointVelocities>(data);
    desired_velocities_.write(velocities);
}

void JointVelocityMode::writeZeroCommand() {
    desired_velocities_.write(franka::JointVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
}
