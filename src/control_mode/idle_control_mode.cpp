#include "idle_control_mode.hpp"


IdleControlMode::IdleControlMode() = default;
IdleControlMode::~IdleControlMode() = default;



protocol::ModeID IdleControlMode::getModeID() const {
    return protocol::ModeID::IDLE;
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
        std::cerr << "[IdleControlMode] Robot or model not set.\n";
        return;
    }
    while (is_running_) {
            try {
                if (robot_) {
                    franka::RobotState state = robot_->readOnce();
                    current_state_->write(state);
                }
            } catch (const franka::Exception& e) {
                std::cerr << "[IdleMode] readOnce() failed: " << e.what() << std::endl;
            }
}
    std::cout << "[IdleControlMode] Exited.\n";
#endif
}

void IdleControlMode::writeCommand(const std::vector<uint8_t>& data) {
    // Idle mode does not process commands
    // std::cout << "[IdleControlMode] Received command data, but idle mode does not accept commands.\n";
}
