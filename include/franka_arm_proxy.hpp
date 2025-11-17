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
#include "utils/service_registry.hpp" 
#include "protocol/codec.hpp"

class FrankaArmProxy {

public:
    // Constructor & Destructor
    explicit FrankaArmProxy(const std::string& config_path);// Constructor that initializes the proxy with a configuration file
    ~FrankaArmProxy();// Destructor to clean up resources

    // Core server operations
    bool start();// Starts the Franka server, initializing the robot and communication sockets
    void stop();// Stops the server, cleaning up resources and shutting down communication
    void spin();// Main loop for processing requests
    std::string getType() const { return type_; } // Returns the type of the proxy (e.g., "Arm" or "Gripper")
    // State management
    void setControlMode(const protocol::FrankaArmControlMode& mode);// Sets the current control mode of the Franka arm
    franka::RobotState getCurrentState(const std::string& request);// Return the current state of the robot
    
private:
    // Initialization
    void initialize(const std::string &filename);// Initializes the FrankaProxy with the given configuration file and set up communication sockets
    // Thread functions
    void statePublishThread();// ZMQ PUB, Publishes the current state of the robot at a fixed rate
    void responseSocketThread();// ZMQ REP,responds to incoming requests from clients
    // Service handler
    void handleServiceRequest(const std::string& service_name, const protocol::ByteView& request, std::vector<uint8_t>& response);



    //Start
    bool startArm();// Starts the arm control loop and initializes the necessary threads
    //Stop
    void stopArm();// Stops the arm control loop and cleans up resources

    void controlLoopThread();// Main control loop for processing commands and updating the robot state
    void stateSubscribeThread();// ZMQ SUB, Subscribes to the state updates from a leader robot (for follower mode)
    void gripperSubscribeThread();// ZMQ SUB, Subscribes to the gripper updates

    
private:
    std::string type_;
    std::string robot_ip_;
    std::string service_addr_;
    std::string state_pub_addr_;
    // Franka robot
    std::shared_ptr<franka::Robot> robot_;
    std::shared_ptr<franka::Model> model_;
    
    // ZMQ communication
    zmq::socket_t state_pub_socket_;//arm state publish socket
    
    // Threading
    std::thread state_pub_thread_;
        
    // Synchronization
    std::atomic<bool> is_running; // for threads
    
    //Control mode
    std::shared_ptr<AbstractControlMode> current_mode_;

    // Current robot state
    AtomicDoubleBuffer<franka::RobotState> current_state_;

    // initialize
    void initializeControlMode();
    void initializeService();

    // service registry
    ServiceRegistry service_registry_;
    franka::RobotState getFrankaArmState();
    uint8_t getFrankaArmControlMode();
    const std::string& getFrankaArmStatePubPort();
    
    // TODO: put all the Constants to a config file
    static constexpr int STATE_PUB_RATE_HZ = 100;
    static constexpr int GRIPPER_PUB_RATE_HZ = 100;
    static constexpr int SOCKET_TIMEOUT_MS = 100;
    static constexpr int MAX_MESSAGE_SIZE = 4096;
};
