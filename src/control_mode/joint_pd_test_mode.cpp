#include "joint_pd_test_mode.hpp"
#include <franka/exception.h>
#include <iostream>

JointPDControlMode::JointPDControlMode() = default;
JointPDControlMode::~JointPDControlMode() = default;

void JointPDControlMode::start() {
    std::cout << "[JointPDControlMode] Started.\n";
    is_running_ = true;

    if (!robot_ || !model_) {
        std::cerr << "[JointPDControlMode] Robot or model not set.\n";
        return;
    }
    robot_->automaticErrorRecovery();
    const std::array<double, 7> Kp = {700.0, 700.0, 700.0, 600.0, 300.0, 300.0, 250.0};
    const std::array<double, 7> Kd = {50.0, 50.0, 50.0, 50.0, 25.0, 25, 15};

    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> torque_callback =
        [this, Kp, Kd](const franka::RobotState& state, franka::Duration) -> franka::Torques {
            if (!is_running_) {
                throw franka::ControlException("JointPDControlMode stopped.");
            }

            setCurrentState(state);

            auto leader_ptr = getLeaderState();
            if (!leader_ptr) {
                std::cerr << "[JointPDControlMode] No leader state available.\n";
                return franka::Torques({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            }

            const franka::RobotState& leader = *leader_ptr;
            std::array<double, 7> tau{};
            for (size_t i = 0; i < 7; ++i) {
                double pos_error = leader.q[i] - state.q[i];
                double vel_error = leader.dq[i] - state.dq[i];
                tau[i] = Kp[i] * pos_error + Kd[i] * vel_error;
            }

            return franka::Torques(tau);
        };

    try {
        robot_->control(torque_callback);
    } catch (const franka::ControlException& e) {
        std::cerr << "[JointPDControlMode] Exception: " << e.what() << std::endl;
    if (std::string(e.what()).find("reflex") != std::string::npos) {
        std::cout << "Reflex detected, attempting automatic recovery...\n";
        try {
            robot_->automaticErrorRecovery();
        } catch (const franka::Exception& recovery_error) {
            std::cerr << "Recovery failed: " << recovery_error.what() << std::endl;
        }
    }
}

    std::cout << "[JointPDControlMode] Exited.\n";
}

void JointPDControlMode::stop() {
    is_running_ = false;
}

int JointPDControlMode::getModeID() const {
    return 6; // Return a unique ID for the Joint PD control mode
}
