#include "cartesian_velocity_mode.hpp"
#include "protocol/mode_id.hpp"
#include "protocol/codec.hpp"
#include <franka/exception.h>
#include <iostream>

CartesianVelocityMode::CartesianVelocityMode():
    desired_velocities_(franka::CartesianVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}})
{};
CartesianVelocityMode::~CartesianVelocityMode() = default;

void CartesianVelocityMode::controlLoop() {
    std::cout << "[CartesianVelocityMode] Started.\n";
    is_running_ = true;

    desired_velocities_.write(franka::CartesianVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

    if (!robot_ || !model_) {
        std::cerr << "[CartesianVelocityMode] Robot or model not set.\n";
        return;
    }

    robot_->automaticErrorRecovery();

    std::function<franka::CartesianVelocities(const franka::RobotState&, franka::Duration)> callback =
        [this](const franka::RobotState& state, franka::Duration) -> franka::CartesianVelocities {
            if (!is_running_) {
                throw franka::ControlException("CartesianVelocityMode stopped.");
            }
            updateRobotState(state);
            auto desired = desired_velocities_.read();
            if (!is_running_) {
                return franka::MotionFinished(desired);
            }
            return desired;
        };
    try {
        robot_->control(callback);
    } catch (const franka::ControlException& e) {
        std::cerr << "[CartesianVelocityMode] Exception: " << e.what() << std::endl;
        if (std::string(e.what()).find("reflex") != std::string::npos) {
            std::cout << "Reflex detected, attempting automatic recovery...\n";
            try {
                robot_->automaticErrorRecovery();
            } catch (const franka::Exception& recovery_error) {
                std::cerr << "Recovery failed: " << recovery_error.what() << std::endl;
            }
        }
        std::cout << "[CartesianVelocityMode] Exited.\n";
    }
}


protocol::ModeID CartesianVelocityMode::getModeID() const {
    return protocol::ModeID::CARTESIAN_VELOCITY;
}

void CartesianVelocityMode::writeCommand(const std::vector<uint8_t>& data) {
    franka::CartesianVelocities velocities = {};
    protocol::decode(data, velocities);
    desired_velocities_.write(velocities);
}
