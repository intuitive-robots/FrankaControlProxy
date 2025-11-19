#include "control_mode/joint_position_mode.hpp"
#include <franka/exception.h>
#include <spdlog/spdlog.h>
#include "protocol/codec.hpp"

JointPositionMode::JointPositionMode():
    desired_positions_(franka::JointPositions{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}})
{};
JointPositionMode::~JointPositionMode() = default;

void JointPositionMode::controlLoop() {
    spdlog::info("[JointPositionMode] Started.");
    is_running_ = true;

    desired_positions_.write(franka::JointPositions{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

    if (!robot_ || !model_) {
        spdlog::error("[JointPositionMode] Robot or model not set.");
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
        spdlog::error("[JointPositionMode] Exception: {}", e.what());
        if (std::string(e.what()).find("reflex") != std::string::npos) {
            spdlog::warn("Reflex detected, attempting automatic recovery...");
            try {
                robot_->automaticErrorRecovery();
            } catch (const franka::Exception& recovery_error) {
                spdlog::error("Recovery failed: {}", recovery_error.what());
            }
        }
        spdlog::info("[JointPositionMode] Exited.");
    }
}


protocol::ModeID JointPositionMode::getModeID() const {
    return protocol::ModeID::JOINT_POSITION;
}

void JointPositionMode::writeCommand(const protocol::ByteView& data) {
    franka::JointPositions positions = protocol::decode<franka::JointPositions>(data);
    desired_positions_.write(positions);
}

void JointPositionMode::writeZeroCommand() {
    franka::JointPositions current_positions = current_state_->read().q;
    desired_positions_.write(current_positions);
}
