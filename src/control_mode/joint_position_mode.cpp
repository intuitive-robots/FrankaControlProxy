#include "joint_position_mode.hpp"
#include <franka/exception.h>
#include <iostream>

JointPositionMode::JointPositionMode() = default;
JointPositionMode::~JointPositionMode() = default;

void JointPositionMode::start() {
    std::cout << "[JointPositionMode] Started.\n";
    is_running_ = true;

    desired_positions_.write(franka::JointPositions{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

    if (!robot_ || !model_) {
        std::cerr << "[JointPositionMode] Robot or model not set.\n";
        return;
    }

    robot_->automaticErrorRecovery();

    std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)> callback =
        [this](const franka::RobotState& state, franka::Duration) -> franka::JointPositions {
            if (!is_running_) {
                throw franka::ControlException("JointPositionMode stopped.");
            }
            updateRobotState(state);
            auto desired = desired_positions_.read();
            if (!is_running_) {
                return franka::MotionFinished(desired);
            }
            return desired;
        };

    try {
        robot_->control(callback);
    } catch (const franka::ControlException& e) {
        std::cerr << "[JointPositionMode] Exception: " << e.what() << std::endl;
        if (std::string(e.what()).find("reflex") != std::string::npos) {
            std::cout << "Reflex detected, attempting automatic recovery...\n";
            try {
                robot_->automaticErrorRecovery();
            } catch (const franka::Exception& recovery_error) {
                std::cerr << "Recovery failed: " << recovery_error.what() << std::endl;
            }
        }
        std::cout << "[JointPositionMode] Exited.\n";
    }
}

void JointPositionMode::stop() {
    is_running_ = false;
    std::cout << "[JointPositionMode] Stopped.\n";
}

int JointPositionMode::getModeID() const {
    return 2;
}
