#include "idle_control_mode.hpp"
#include <spdlog/spdlog.h>


IdleControlMode::IdleControlMode() = default;
IdleControlMode::~IdleControlMode() = default;



protocol::ModeID IdleControlMode::getModeID() const {
    return protocol::ModeID::IDLE;
}

    void IdleControlMode::start() {
        startRobot();
        control_thread_ = std::thread(&IdleControlMode::controlLoop, this);
        spdlog::info("[{}] Control thread launched.", getModeName());
    }


void IdleControlMode::controlLoop() {
#if LOCAL_TESTING
    while (is_running_) {
        franka::RobotState state = franka::RobotState{};
        current_state_->write(state);
        // In local testing, just keep the loop alive
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#else
    if (!robot_ || !model_) {
        spdlog::error("[IdleControlMode] Robot or model not set.");
        return;
    }
    while (is_running_) {
            try {
                if (robot_) {
                    franka::RobotState state = robot_->readOnce();
                    current_state_->write(state);
                }
            } catch (const franka::Exception& e) {
                spdlog::error("[IdleMode] readOnce() failed: {}", e.what());
            }
}
    spdlog::info("[IdleControlMode] Exited.");
#endif
}

void IdleControlMode::writeCommand(const protocol::ByteView& data) {
    // Idle mode does not process commands
    // std::cout << "[IdleControlMode] Received command data, but idle mode does not accept commands.\n";
}

void IdleControlMode::writeZeroCommand() {
    // No action needed for zero command in idle mode
}
