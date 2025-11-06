#include "cartesian_position_mode.hpp"
#include <franka/exception.h>
#include <iostream>

CartesianPositionMode::CartesianPositionMode() = default;
CartesianPositionMode::~CartesianPositionMode() = default;

void CartesianPositionMode::start() {
    std::cout << "[CartesianPositionMode] Started.\n";
    is_running_ = true;

    // Initialize desired Cartesian pose to identity (no movement)
    desired_pose_.write(franka::CartesianPose{
        {1.0, 0.0, 0.0, 0.0,   // Row 1 (rotation)
         0.0, 1.0, 0.0, 0.0,   // Row 2 (rotation)
         0.0, 0.0, 1.0, 0.0,   // Row 3 (rotation)
         0.0, 0.0, 0.0, 1.0}   // Row 4 (translation, homogeneous)
    });

    if (!robot_ || !model_) {
        std::cerr << "[CartesianPositionMode] Robot or model not set.\n";
        return;
    }

    robot_->automaticErrorRecovery();

    std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)> callback =
        [this](const franka::RobotState& state, franka::Duration) -> franka::CartesianPose {
            if (!is_running_) {
                throw franka::ControlException("CartesianPositionMode stopped.");
            }

            updateRobotState(state);
            auto desired = desired_pose_.read();

            if (!is_running_) {
                return franka::MotionFinished(desired);
            }

            return desired;
        };

    try {
        robot_->control(callback);
    } catch (const franka::ControlException& e) {
        std::cerr << "[CartesianPositionMode] Exception: " << e.what() << std::endl;
        if (std::string(e.what()).find("reflex") != std::string::npos) {
            std::cout << "Reflex detected, attempting automatic recovery...\n";
            try {
                robot_->automaticErrorRecovery();
            } catch (const franka::Exception& recovery_error) {
                std::cerr << "Recovery failed: " << recovery_error.what() << std::endl;
            }
        }
        std::cout << "[CartesianPositionMode] Exited.\n";
    }
}

void CartesianPositionMode::stop() {
    is_running_ = false;
    std::cout << "[CartesianPositionMode] Stopped.\n";
}

int CartesianPositionMode::getModeID() const {
    return 5;
}
