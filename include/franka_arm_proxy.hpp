#pragma once
#include <zmq.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <memory>

#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include "control_mode/abstract_control_mode.hpp"
#include "utils/atomic_double_buffer.hpp"
#include "utils/config_file_reader.hpp"
#include "protocol/msg.hpp"
#include <zerolancom/zerolancom.hpp>


struct FrankaArmConfig {
    // communication
    std::string robot_ip;
    std::string state_pub_addr;
    std::string service_addr;

    // arm
    std::array<double, 7> arm_default_state_q;
    std::array<double, 16> arm_default_state_O_T_EE{};
    int arm_state_pub_rate_hz{100};
    int arm_socket_timeout_ms{100};
    int arm_max_message_size{4096};
    // collision behavior thresholds
    std::array<double, 7> arm_col_lower_torque_acc{};
    std::array<double, 7> arm_col_upper_torque_acc{};
    std::array<double, 7> arm_col_lower_torque_nom{};
    std::array<double, 7> arm_col_upper_torque_nom{};
    std::array<double, 6> arm_col_lower_force_acc{};
    std::array<double, 6> arm_col_upper_force_acc{};
    std::array<double, 6> arm_col_lower_force_nom{};
    std::array<double, 6> arm_col_upper_force_nom{};

    FrankaArmConfig(const std::string& arm_config_path) {
        fromFile(arm_config_path);
    }


    void fromFile(const std::string& arm_config_path) {
        ConfigFileReader reader(arm_config_path);
        // defaults for arrays
        const std::array<double, 16> default_O_T_EE{{
            1.0, 0.0, 0.0, 0.3,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.5,
            0.0, 0.0, 0.0, 1.0
        }};

        // communication
        robot_ip = reader.getValue<std::string>("robot_ip", "");
        state_pub_addr = reader.getValue<std::string>("state_pub_addr", "");
        service_addr = reader.getValue<std::string>("service_addr", "");

        // arm
        arm_default_state_q = reader.getArray<7>("arm_default_state_q", {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785});

        arm_default_state_O_T_EE = reader.getArray<16>("arm_default_state_O_T_EE", default_O_T_EE);
        arm_state_pub_rate_hz = reader.getValue<int>("arm_state_pub_rate_hz", 100);
        arm_socket_timeout_ms = reader.getValue<int>("arm_socket_timeout_ms", 100);
        arm_max_message_size = reader.getValue<int>("arm_max_message_size", 4096);

        // collision behavior (defaults simple non-zero thresholds to avoid 0)
        const std::array<double, 7> d7_acc_low{{30, 30, 30, 30, 30, 30, 30}};
        const std::array<double, 7> d7_acc_up {{45, 45, 45, 45, 45, 45, 45}};
        const std::array<double, 7> d7_nom_low{{25, 25, 25, 25, 25, 25, 25}};
        const std::array<double, 7> d7_nom_up {{35, 35, 35, 35, 35, 35, 35}};
        const std::array<double, 6> d6_acc_low{{10, 10, 10, 10, 10, 10}};
        const std::array<double, 6> d6_acc_up {{15, 15, 15, 15, 15, 15}};
        const std::array<double, 6> d6_nom_low{{8, 8, 8, 8, 8, 8}};
        const std::array<double, 6> d6_nom_up {{12, 12, 12, 12, 12, 12}};

        arm_col_lower_torque_acc = reader.getArray<7>("arm_collision_lower_torque_thresholds_acc", d7_acc_low);
        arm_col_upper_torque_acc = reader.getArray<7>("arm_collision_upper_torque_thresholds_acc", d7_acc_up);
        arm_col_lower_torque_nom = reader.getArray<7>("arm_collision_lower_torque_thresholds_nom", d7_nom_low);
        arm_col_upper_torque_nom = reader.getArray<7>("arm_collision_upper_torque_thresholds_nom", d7_nom_up);
        arm_col_lower_force_acc  = reader.getArray<6>("arm_collision_lower_force_thresholds_acc", d6_acc_low);
        arm_col_upper_force_acc  = reader.getArray<6>("arm_collision_upper_force_thresholds_acc", d6_acc_up);
        arm_col_lower_force_nom  = reader.getArray<6>("arm_collision_lower_force_thresholds_nom", d6_nom_low);
        arm_col_upper_force_nom  = reader.getArray<6>("arm_collision_upper_force_thresholds_nom", d6_nom_up);
        }

};


class FrankaArmProxy {

public:
    // Constructor & Destructor
    explicit FrankaArmProxy(const std::string& config_path);// Constructor that initializes the proxy with a configuration file
    ~FrankaArmProxy();// Destructor to clean up resources

    // Core server operations
    void stop(); // Stops the server, cleaning up resources and shutting down communication
    void spin(); // Main loop for processing requests
    // State management
    zlc::Empty setControlMode(const std::string& mode);// Sets the current control mode of the Franka arm
    franka::RobotState getCurrentState(const std::string& request);// Return the current state of the robot
    
private:
    // Initialization
    void initRobot();

private:
    // Franka robot
    std::shared_ptr<franka::Robot> robot_;
    std::shared_ptr<franka::Model> model_;
    
    // Threading
    std::thread state_pub_thread;

    // Control modes registry
    std::unordered_map<std::string, std::shared_ptr<AbstractControlMode>> control_modes_;

    // Synchronization
    std::atomic<bool> is_running; // for threads
    
    //Control mode
    std::shared_ptr<AbstractControlMode> current_control_mode_;

    // Current robot state
    AtomicDoubleBuffer<franka::RobotState> current_state;

    FrankaArmConfig config_;

    // initialize
    void initializeControlMode();
    void initializeService();

    // Service callbacks
    FrankaArmState getFrankaArmState(const zlc::Empty&);// Gets the current state of the Franka arm
    std::string getFrankaArmControlMode(const zlc::Empty&);

    void statePublishThread();

};
