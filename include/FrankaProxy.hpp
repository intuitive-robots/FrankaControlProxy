#ifndef FRANKA_SERVER_HPP
#define FRANKA_SERVER_HPP

#include <zmq.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <memory>

#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/gripper.h>
#include <yaml-cpp/yaml.h>
#include "protocol/franka_arm_state.hpp"
#include "control_mode/abstract_control_mode.h"
#include "protocol/franka_gripper_state.hpp"

class FrankaProxy {

public:
    // Constructor & Destructor
    explicit FrankaProxy(const std::string& config_path);// Constructor that initializes the proxy with a configuration file
    ~FrankaProxy();// Destructor to clean up resources

    // Core server operations
    bool start();// Starts the Franka server, initializing the robot and communication sockets
    void stop();// Stops the server, cleaning up resources and shutting down communication
    void spin();// Main loop for processing requests
    std::string getType() const { return type_; } // Returns the type of the proxy (e.g., "Arm" or "Gripper")
    // State management
    franka::RobotState getCurrentState();// Return the current state of the robot
    protocol::FrankaGripperState getCurrentGripper();// Return the current state of the gripper
    // Mode management
    void registerControlMode(const std::string& mode, std::unique_ptr<AbstractControlMode> control_mode);//register the map
    void setControlMode(const std::string& mode);
    
    
    
    // Configuration
    void displayConfig() const;
    
private:
    // Initialization
    void initialize(const std::string &filename);// Initializes the FrankaProxy with the given configuration file and set up communication sockets
    //Start
    bool startArm();// Starts the arm control loop and initializes the necessary threads
    bool startGripper();// Starts the gripper control loop and initializes the necessary threads
    //Stop
    void stopArm();// Stops the arm control loop and cleans up resources
    void stopGripper();// Stops the gripper control loop and cleans up resources
    // Thread functions
    void statePublishThread();// ZMQ PUB, Publishes the current state of the robot at a fixed rate
    void gripperPublishThread();// ZMQ PUB, Publishes the current gripper state at a fixed rate
    void responseSocketThread();// ZMQ REP,responds to incoming requests from clients
    void controlLoopThread();// Main control loop for processing commands and updating the robot state
    void stateSubscribeThread();// ZMQ SUB, Subscribes to the state updates from a leader robot (for follower mode)
    void gripperSubscribeThread();// ZMQ SUB, Subscribes to the gripper updates
    // Message handling
    void handleServiceRequest(const std::vector<uint8_t>& request, std::vector<uint8_t>& response) ;
    
    
private:
    // Configuration
    YAML::Node proxy_config_;
    std::string type_;
    std::string robot_ip_;
    std::string service_addr_;
    std::string state_pub_addr_;
    std::string gripper_pub_addr_;
    std::string state_sub_addr_;
    std::string gripper_sub_addr_;
    std::string follower_ = "false";
    // Franka robot
    std::shared_ptr<franka::Robot> robot_;
    std::shared_ptr<franka::Model> model_;
    //Franka gripper
    std::shared_ptr<franka::Gripper> gripper_;
    
    // ZMQ communication
    zmq::context_t context_;
    zmq::socket_t pub_arm_socket_;//arm state publish socket
    zmq::socket_t sub_cmd_socket_;//command socket
    zmq::socket_t rep_socket_;//service socket
    //zmq::socket_t pub_socket_;
    // zmq::socket_t pub_gripper_socket_;
    //zmq::socket_t sub_socket_;
    // zmq::socket_t sub_gripper_socket_;
    
    
    // Threading
    std::thread state_pub_thread_;//statePublishThread()
    // std::thread state_sub_thread_;//stateSubscribeThread(),just for follower
    std::thread command_sub_thread_;//commandSubscribeThread(),just for follower
    std::thread control_thread_;//controlLoopThread()
    std::thread service_thread_;//responseSocketThread()
    
    // std::thread gripper_pub_thread_;//gripperPublishThread()

    

    
    // std::thread gripper_sub_thread_;//gripperSubscribeThread(),just for follower
    
    
    // Synchronization
    std::atomic<bool> in_control_;//for threads
    
    // State data
    franka::RobotState current_state_;
    franka::RobotState leader_state_; //for follower to generate control
    // Gripper data
    franka::GripperState current_gripper_state_;
    franka::GripperState leader_gripper_state_; //for follower
    
    //Control mode
    AbstractControlMode* current_mode_ = nullptr;
    std::mutex control_mutex_;
    std::map<std::string, std::shared_ptr<AbstractControlMode>> control_modes_map_;
    
    // TODO: put all the Constants to a config file
    static constexpr int STATE_PUB_RATE_HZ = 100;
    static constexpr int GRIPPER_PUB_RATE_HZ = 100;
    static constexpr int SOCKET_TIMEOUT_MS = 100;
    static constexpr int MAX_MESSAGE_SIZE = 4096;
};

#endif // FRANKA_SERVER_HPP