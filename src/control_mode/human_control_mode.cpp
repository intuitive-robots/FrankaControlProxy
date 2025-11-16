
#include "human_control_mode.hpp"
#include <franka/exception.h>
#include <iostream>

HumanControlMode::HumanControlMode() = default;
HumanControlMode::~HumanControlMode() = default;

void HumanControlMode::start() {
    is_running_ = true;
    std::cout << "[HumanControlMode] Started.\n";
    if (!robot_ || !model_) {
        std::cerr << "[HumanControlMode] Robot or model not set.\n";
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
            std::cerr << "[HumanControlMode] Exception: " << e.what() << std::endl;
        if (std::string(e.what()).find("reflex") != std::string::npos) {
            std::cout << "Reflex detected, attempting automatic recovery...\n";
            try {
                robot_->automaticErrorRecovery();
            } catch (const franka::Exception& recovery_error) {
                std::cerr << "Recovery failed: " << recovery_error.what() << std::endl;
            }
        }
        std::cout << "[HumanControlMode] Exited.\n";
    }
}

const std::string& HumanControlMode::getModeName() {
    return ModeID::HUMAN_CONTROL;
}


void HumanControlMode::writeCommand(const std::vector<uint8_t>& data) {
    // HumanControlMode does not process external commands; ignore incoming data.
    std::cout << "[HumanControlMode] Received command data, but this mode does not accept commands.\n";
}

void HumanControlMode::controlLoop() {
    // HumanControlMode control logic is handled in start(); this function is unused.
    std::cout << "[HumanControlMode] controlLoop() called, but control is managed in start().\n";
}