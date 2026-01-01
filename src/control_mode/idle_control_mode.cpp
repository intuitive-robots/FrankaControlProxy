#include "control_mode/idle_control_mode.hpp"



IdleControlMode::IdleControlMode() = default;
IdleControlMode::~IdleControlMode() = default;



void IdleControlMode::initController() {
    zlc::info("[{}] Initializing Idle Control Mode.", getModeName());
}


void IdleControlMode::controlLoop() {
    while (is_running_) {
            try {
                if (robot_) {
                    franka::RobotState state = robot_->readOnce();
                    state_buffer_->write(state);
                }
            } catch (const franka::Exception& e) {
                zlc::error("[IdleMode] readOnce() failed: {}", e.what());
            }
    }
    zlc::info("[IdleControlMode] Exited.");
}