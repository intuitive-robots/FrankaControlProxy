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
#include <yaml-cpp/yaml.h>
#include "control_mode/abstract_control_mode.hpp"
#include "utils/atomic_double_buffer.hpp"
#include "utils/franka_config.hpp"
#include "protocol/codec.hpp"
#include <zerolancom/zerolancom.hpp>

class FrankaArmProxy {

public:
    // Constructor & Destructor
    explicit FrankaArmProxy(const std::string& config_path);// Constructor that initializes the proxy with a configuration file
    ~FrankaArmProxy();// Destructor to clean up resources

    // Core server operations
    bool start(); // Starts the Franka server, initializing the robot and communication sockets
    void stop(); // Stops the server, cleaning up resources and shutting down communication
    void spin(); // Main loop for processing requests
    // State management
    void setControlMode(const std::string& mode);// Sets the current control mode of the Franka arm
    franka::RobotState getCurrentState(const std::string& request);// Return the current state of the robot
    
private:
    // Initialization
    void initialize(const std::string &filename);// Initializes the FrankaArmProxy with the given configuration file and set up communication sockets
    //Start
    bool start();// Starts the arm control loop and initializes the necessary threads
    //Stop
    void stop();// Stops the arm control loop and cleans up resources


private:
    std::string type_;
    std::string robot_ip_;
    // Franka robot
    std::shared_ptr<franka::Robot> robot_;
    std::shared_ptr<franka::Model> model_;
    
    // Threading
    std::thread state_pub_thread;
    std::unordered_map<std::string, std::unique_ptr<AbstractControlMode>> control_modes_;

    // Synchronization
    std::atomic<bool> is_running; // for threads
    
    //Control mode
    std::shared_ptr<AbstractControlMode> current_mode_;

    // Current robot state
    AtomicDoubleBuffer<franka::RobotState> current_state;

    FrankaArmConfig config_;
    franka::RobotState default_state_;

    // initialize
    void initializeControlMode();
    void initializeService();

    // Service callbacks
    franka::RobotState getFrankaArmState();
    uint8_t getFrankaArmControlMode();

    void statePublishThread();

    // TODO: put all the Constants to a config file
    static constexpr int STATE_PUB_RATE_HZ = 100;
    static constexpr int GRIPPER_PUB_RATE_HZ = 100;
    static constexpr int SOCKET_TIMEOUT_MS = 100;
    static constexpr int MAX_MESSAGE_SIZE = 4096;
};
