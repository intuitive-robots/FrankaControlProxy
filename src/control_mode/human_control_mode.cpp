
#include "human_control_mode.hpp"
#include <franka/exception.h>
#include "utils/logger.hpp"

HumanControlMode::HumanControlMode() = default;
HumanControlMode::~HumanControlMode() = default;

void HumanControlMode::start() {
    is_running_ = true;
    LOG_INFO("[HumanControlMode] Started.");
    if (!robot_ || !model_) {
        LOG_ERROR("[HumanControlMode] Robot or model not set.");
        return;
    }
    robot_->setCollisionBehavior(
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    robot_->automaticErrorRecovery();
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> callback =
        [this](const franka::RobotState& state, franka::Duration) -> franka::Torques {
            updateRobotState(state);
            std::array<double, 7> damping_torque{};
            for (size_t i = 0; i < 7; ++i) {
                damping_torque[i] = -0.5 * state.dq[i];  // 0.5 Nm·s/rad
            }
            franka::Torques torques = franka::Torques(damping_torque);
            if (!is_running_) {
                return franka::MotionFinished(torques);
            }
            return torques;
        };

    try {
        robot_->control(callback);
    } catch (const franka::ControlException& e) {
            LOG_ERROR("[HumanControlMode] Exception: {}", e.what());
        if (std::string(e.what()).find("reflex") != std::string::npos) {
            LOG_WARN("Reflex detected, attempting automatic recovery...");
            try {
                robot_->automaticErrorRecovery();
            } catch (const franka::Exception& recovery_error) {
                LOG_ERROR("Recovery failed: {}", recovery_error.what());
            }
        }
        LOG_INFO("[HumanControlMode] Exited.");
    }
}

protocol::ModeID HumanControlMode::getModeID() const {
    return protocol::ModeID::HUMAN_CONTROL;
}


void HumanControlMode::writeCommand(const protocol::ByteView& data) {
    // HumanControlMode does not process external commands; ignore incoming data.
    LOG_WARN("[HumanControlMode] Received command data, but this mode does not accept commands.");
}

void HumanControlMode::controlLoop() {
    // HumanControlMode control logic is handled in start(); this function is unused.
    LOG_WARN("[HumanControlMode] controlLoop() called, but control is managed in start().");
}

void HumanControlMode::writeZeroCommand() {
    // No action needed for zero command in human control mode
}
