#include "control_mode/cartesian_pose_mode.hpp"
#include "protocol/mode_id.hpp"
#include "protocol/codec.hpp"
#include <franka/exception.h>
#include <iostream>

CartesianPoseMode::CartesianPoseMode():
    desired_pose_(franka::CartesianPose{
        {1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0}
    })
{};
CartesianPoseMode::~CartesianPoseMode() = default;

void CartesianPoseMode::controlLoop() {
    std::cout << "[CartesianPoseMode] Started.\n";
    is_running_ = true;

    // Initialize desired Cartesian pose to identity (no movement)`
    desired_pose_.write(franka::CartesianPose{
        {1.0, 0.0, 0.0, 0.0,   // Row 1 (rotation)
         0.0, 1.0, 0.0, 0.0,   // Row 2 (rotation)
         0.0, 0.0, 1.0, 0.0,   // Row 3 (rotation)
         0.0, 0.0, 0.0, 1.0}   // Row 4 (translation, homogeneous)
    });

    if (!robot_ || !model_) {
        std::cerr << "[CartesianPoseMode] Robot or model not set.\n";
        return;
    }

    robot_->automaticErrorRecovery();

    std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)> callback =
        [this](const franka::RobotState& state, franka::Duration) -> franka::CartesianPose {
            if (!is_running_) {
                throw franka::ControlException("CartesianPoseMode stopped.");
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
        std::cerr << "[CartesianPoseMode] Exception: " << e.what() << std::endl;
        if (std::string(e.what()).find("reflex") != std::string::npos) {
            std::cout << "Reflex detected, attempting automatic recovery...\n";
            try {
                robot_->automaticErrorRecovery();
            } catch (const franka::Exception& recovery_error) {
                std::cerr << "Recovery failed: " << recovery_error.what() << std::endl;
            }
        }
        std::cout << "[CartesianPoseMode] Exited.\n";
    }
}

protocol::ModeID CartesianPoseMode::getModeID() const {
    return protocol::ModeID::CARTESIAN_POSE;
}

void CartesianPoseMode::writeCommand(const protocol::ByteView& data) {
    franka::CartesianPose pose = protocol::decode<franka::CartesianPose>(data);
    desired_pose_.write(pose);
}

void CartesianPoseMode::writeZeroCommand() {
    franka::CartesianPose current_pose = current_state_->read().O_T_EE;
    desired_pose_.write(current_pose);
}