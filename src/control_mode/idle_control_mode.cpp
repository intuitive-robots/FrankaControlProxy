#include "idle_control_mode.hpp"


IdleControlMode::IdleControlMode() = default;
IdleControlMode::~IdleControlMode() = default;


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
                    current_state_->write(state);
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

const std::string& IdleControlMode::getModeName() const {
    return protocol::toString(protocol::ModeID::IDLE);
}

void IdleControlMode::controlLoop() {
    // Idle mode does not have a control loop
    std::cout << "[IdleControlMode] controlLoop() called, but idle mode has no control loop.\n";
}

void IdleControlMode::writeCommand(const std::vector<uint8_t>& data) {
    // Idle mode does not process commands
    std::cout << "[IdleControlMode] Received command data, but idle mode does not accept commands.\n";
}
