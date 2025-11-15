#pragma once
#include <zmq.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <memory>

#include <franka/gripper.h>
#include <franka/robot_state.h>
#include <yaml-cpp/yaml.h>
#include "control_mode/abstract_control_mode.hpp"
#include "utils/atomic_double_buffer.hpp"
#include "utils/service_registry.hpp" 
#include "protocol/codec.hpp"
#include "protocol/grasp_command.hpp"

class FrankaGripperProxy {

public:
    // Constructor & Destructor
    explicit FrankaGripperProxy(const std::string& config_path);// Constructor that initializes the proxy with a configuration file
    ~FrankaGripperProxy();// Destructor to clean up resources

    // Core server operations
    bool start();

private:
    // Initialization
    void initialize(const std::string &filename);// Initializes the FrankaProxy with the given configuration file and set up communication sockets
    // Thread functions
    void statePublishThread();// ZMQ PUB, Publishes the current state of the robot at a fixed rate
    bool setGripperWidth(const GraspCommand& command) {
        try {
            gripper_->grasp(command.width, command.speed, command.force);
            return true;
        } catch (const franka::Exception& e) {
            std::cerr << "[FrankaGripperProxy] Grasp failed: " << e.what() << std::endl;
            return false;
        }
    }

private:
    // Configuration
    YAML::Node proxy_config_;
    std::string type_;
    std::string robot_ip_;
    std::string service_addr_;
    std::string state_pub_addr_;
    // Franka robot
    std::shared_ptr<franka::Gripper> gripper_;
    // ZMQ communication
    zmq::context_t context_;
    zmq::socket_t state_pub_socket_;//arm state publish socket
    zmq::socket_t res_socket_;//service socket
    
    // Threading
    std::thread state_pub_thread_;
    // Synchronization
    std::atomic<bool> is_running; // for threads
    

    // service registry
    ServiceRegistry service_registry_;
    franka::GripperState getFrankaGripperState();
    const std::string& getFrankaGripperStatePubPort();
    
    // TODO: put all the Constants to a config file
    static constexpr int STATE_PUB_RATE_HZ = 100;
    static constexpr int GRIPPER_PUB_RATE_HZ = 100;
    static constexpr int SOCKET_TIMEOUT_MS = 100;
    static constexpr int MAX_MESSAGE_SIZE = 4096;
};
