#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <mutex>
#include <string>
#include <zerolancom/zerolancom.hpp>

#include "control_mode/control_mode.hpp"
#include "protocol/msg.hpp"
#include "protocol/request_result.hpp"
#include "utils/atomic_double_buffer.hpp"
#include "utils/config_file_reader.hpp"
#include "utils/robot_model.hpp"

struct FrankaArmConfig
{
    // communication
    std::string name;
    std::string robot_ip;

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

    FrankaArmConfig(const std::string& arm_config_path)
    {
        fromFile(arm_config_path);
    }

    void fromFile(const std::string& arm_config_path)
    {
        ConfigFileReader reader(arm_config_path);
        // defaults for arrays
        const std::array<double, 16> default_O_T_EE{
            {1.0, 0.0, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.0, 0.0, 1.0}};

        // communication
        name = reader.getValue<std::string>("name");
        robot_ip = reader.getValue<std::string>("robot_ip");
        // arm
        arm_default_state_q = reader.getArray<double, 7>("arm_default_state_q");

        arm_default_state_O_T_EE = reader.getArray<double, 16>("arm_default_state_O_T_EE");
        arm_state_pub_rate_hz = reader.getValue<int>("arm_state_pub_rate_hz");
        arm_socket_timeout_ms = reader.getValue<int>("arm_socket_timeout_ms");
        arm_max_message_size = reader.getValue<int>("arm_max_message_size");

        arm_col_lower_torque_acc =
            reader.getArray<double, 7>("arm_collision_lower_torque_thresholds_acc");
        arm_col_upper_torque_acc =
            reader.getArray<double, 7>("arm_collision_upper_torque_thresholds_acc");
        arm_col_lower_torque_nom =
            reader.getArray<double, 7>("arm_collision_lower_torque_thresholds_nom");
        arm_col_upper_torque_nom =
            reader.getArray<double, 7>("arm_collision_upper_torque_thresholds_nom");
        arm_col_lower_force_acc =
            reader.getArray<double, 6>("arm_collision_lower_force_thresholds_acc");
        arm_col_upper_force_acc =
            reader.getArray<double, 6>("arm_collision_upper_force_thresholds_acc");
        arm_col_lower_force_nom =
            reader.getArray<double, 6>("arm_collision_lower_force_thresholds_nom");
        arm_col_upper_force_nom =
            reader.getArray<double, 6>("arm_collision_upper_force_thresholds_nom");
    }
};

class PandaArm
{
  public:
    // Constructor & Destructor
    explicit PandaArm(
        const std::string&
            config_path); // Constructor that initializes the proxy with a configuration file
    ~PandaArm();    // Destructor to clean up resources

    // Core server operations
    void stop(); // Stops the server, cleaning up resources and shutting down communication
    void spin(); // Main loop for processing requests
    // State management
    zlc::Empty setControlMode(
        const std::string& mode); // Sets the current control mode of the Franka arm
    franka::RobotState getCurrentState(
        const std::string& request); // Return the current state of the robot

  private:
    // Initialization
    void initRobot();
    SafetyLimitConfig safety_config_;
    // Franka robot
    std::unique_ptr<FrankaPanda> robot_;
    std::unique_ptr<PandaPinocchioModel> model_;

    // Threading
    std::thread state_pub_thread;

    // Control modes registry
    std::unordered_map<std::string, std::unique_ptr<AbstractControlMode>> control_modes_;

    // Synchronization
    std::atomic<bool> is_running; // for threads

    //Control mode
    AbstractControlMode* current_control_mode_ = nullptr;

    // Current robot state
    AtomicDoubleBuffer<franka::RobotState> current_state;

    FrankaArmConfig config_;

    // initialize
    void initializeControlMode();
    void initializeService();

    // Service callbacks
    FrankaArmState getFrankaArmState(const zlc::Empty&); // Gets the current state of the Franka arm
    std::string getFrankaArmControlMode(const zlc::Empty&);
    std::pair<std::string, std::vector<uint8_t>> moveFrankaArmToJointPosition(
        const std::vector<double>& target_q);

    void statePublishThread();
};