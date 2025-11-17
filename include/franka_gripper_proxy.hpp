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
#include "utils/franka_config.hpp"
#include "protocol/codec.hpp"
#include "protocol/grasp_command.hpp"

class FrankaGripperProxy {

public:
    // Constructor & Destructor
    explicit FrankaGripperProxy(const std::string& config_path) {
        FrankaConfig config(config_path);
        gripper_ip_ = config.getValue("gripper_ip");
        std::cout<<"gripper ip: "<< gripper_ip_ <<std::endl;
        gripper_ = std::make_shared<franka::Gripper>(gripper_ip_);
        gripper_->homing();
        //bind state pub socket
        state_pub_addr_ = config.getValue("gripper_state_pub_addr");
        std::cout<<"gripper state_pub: "<< state_pub_addr_ <<std::endl;
        state_pub_socket_ = zmq::socket_t(ZmqContext::instance(), ZMQ_PUB);
        state_pub_socket_.bind(state_pub_addr_);
        service_registry_.bindSocket(config.getValue("gripper_service_addr"));
        //initialize franka gripper
        initializeService();
    };
    ~FrankaGripperProxy() {
        stop();
    };

    // Core server operations
    void start() {
        is_running = true;
        std::cout << is_running <<"gripper control"<< std::endl;
        // state_pub_thread_ = std::thread(&FrankaGripperProxy::statePublishThread, this);
        service_registry_.start();
    };

    void stop() {
        std::cout << "[INFO] Stopping FrankaGripperProxy..." << std::endl;
        is_running = false;
        if (state_pub_thread_.joinable()) state_pub_thread_.join();
        // try close ZeroMQ sockets
        try {
            state_pub_socket_.close();
        } catch (const zmq::error_t& e) {
            std::cerr << "[ZMQ ERROR] " << e.what() << "\n";
        }
        // wait for closing
        service_registry_.stop();
        gripper_.reset();
        std::cout << "[INFO] FrankaGripperProxy stopped successfully." << std::endl;
    };

private:
    // Initialization
    void initializeService() {
        // Register service handlers
        service_registry_.registerHandler("SET_FRANKA_GRIPPER_WIDTH", this, &FrankaGripperProxy::setGripperWidth);
        // service_registry_.registerHandler("GET_FRANKA_GRIPPER_STATE", this, &FrankaGripperProxy::getFrankaGripperState);
        // service_registry_.registerHandler("GET_FRANKA_GRIPPER_STATE_PUB_PORT", this, &FrankaGripperProxy::getFrankaGripperStatePubPort);
    };
    // Thread functions
    // void statePublishThread();
    void setGripperWidth(const protocol::GraspCommand& command) {
# if LOCAL_TESTING
        std::cout << "[FrankaGripperProxy] Setting gripper width to " << command.width
                  << " with speed " << command.speed << " and force " << command.force << std::endl;
# else
        std::cout << "[FrankaGripperProxy] Setting gripper width to " << command.width
                  << " with speed " << command.speed << " and force " << command.force << std::endl;
        try {
            gripper_->stop();
            gripper_->grasp(command.width, command.speed, 1);
        } catch (const franka::Exception& e) {
            std::cerr << "[FrankaGripperProxy] Grasp failed: " << e.what() << std::endl;
        }
# endif
    };

    std::string gripper_ip_;
    std::string service_addr_;
    std::string state_pub_addr_;
    // Franka robot
    std::shared_ptr<franka::Gripper> gripper_;
    // ZMQ communication
    zmq::socket_t state_pub_socket_;//arm state publish socket

    
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
