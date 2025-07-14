#include "idle_control_mode.hpp"

// void IdleControlMode::initialize(const franka::RobotState& initial_state) {
//     //todo:write recover
    
//     current_state_ = initial_state;
//     std::cout << "[IdleControlMode] Initialized with initial state." << std::endl;
// }
void IdleControlMode::start() {
    std::cout << "[IdleControlMode] Entering idle mode. Reading state once..." << std::endl;
    is_running_ = true;
    while (is_running_) {
            try {
                if (robot_) {
                    franka::RobotState state = robot_->readOnce();
                    setCurrentState(state);
                }
            } catch (const franka::Exception& e) {
                std::cerr << "[IdleMode] readOnce() failed: " << e.what() << std::endl;
            }
        //     // test get leader state
        //     auto leader_ptr = getLeaderState();
        //     if (leader_ptr) {
        //     const auto& leader = *leader_ptr;

        //     // print leader state
        //     std::cout << "[Leader q] ";
        //     for (double q_i : leader.q) {
        //         std::cout << q_i << " ";
        //     }
        //     std::cout << std::endl;
        // } else {
        //     std::cout << "No leader state available." << std::endl;
        // }
}
    std::cout << "[IdleControlMode] Exited.\n";
}
void IdleControlMode::stop() {
    is_running_ = false;
    std::cout << "[IdleControlMode] Stopping idle mode." << std::endl;
}
int IdleControlMode::getModeID() const {
    return 5; // Return a unique ID for the idle mode
}
