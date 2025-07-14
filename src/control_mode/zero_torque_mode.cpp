
#include "zero_torque_mode.hpp"
#include <franka/exception.h>
#include <iostream>
void ZeroTorqueMode::start() {
    is_running_ = true;
    std::cout << "[ZeroTorqueMode] Started.\n";
    if (!robot_ || !model_) {
        std::cerr << "[ZeroTorqueMode] Robot or model not set.\n";
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
            if (!is_running_) {
                return franka::Torques({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            }
            setCurrentState(state);
            std::array<double, 7> damping_torque{};
            for (size_t i = 0; i < 7; ++i) {
                damping_torque[i] = -0.5 * state.dq[i];  // 0.5 Nm·s/rad
            }
            return franka::Torques(damping_torque);
        };
    try {
        robot_->control(callback);
    } catch (const franka::ControlException& e) {
        std::cerr << "[ZeroTorqueMode] Exception: " << e.what() << std::endl;
    if (std::string(e.what()).find("reflex") != std::string::npos) {
        std::cout << "Reflex detected, attempting automatic recovery...\n";
        try {
            robot_->automaticErrorRecovery();
        } catch (const franka::Exception& recovery_error) {
            std::cerr << "Recovery failed: " << recovery_error.what() << std::endl;
        }
    }
}
    std::cout << "[ZeroTorqueMode] Exited.\n";
}
void ZeroTorqueMode::stop() {
    is_running_ = false;
    std::cout << "[ZeroTorqueMode] Stopping Zero Torque mode.\n";
}
int ZeroTorqueMode::getModeID() const {
    return 4;
}