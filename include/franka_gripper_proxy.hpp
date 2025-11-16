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
#include "generated/message_generated.h"

class FrankaGripperProxy {

public:
    // Constructor & Destructor
    explicit FrankaGripperProxy(const std::string& config_path);// Constructor that initializes the proxy with a configuration file
    ~FrankaGripperProxy();// Destructor to clean up resources

    // Core server operations
    bool start();
    void stop() {
        is_running.store(false);
        if (state_pub_thread_.joinable()) state_pub_thread_.join();
        // try close ZeroMQ sockets
        try {
            state_pub_socket_.close();
        } catch (const zmq::error_t& e) {
            std::cerr << "[ZMQ ERROR] " << e.what() << "\n";
        }
        gripper_->stop();
    };

private:
    // Initialization
    void initialize(const std::string &filename) {
        FrankaConfig config(filename);
        // type_ = config.getValue("type", "Arm"); // Default to "Arm" if not specified
        gripper_ip_ = config.getValue("gripper_ip");
        //bind state pub socket
        state_pub_addr_ = config.getValue("state_pub_addr");
        std::cout<<"state_pub: "<< state_pub_addr_ <<std::endl;
        state_pub_socket_.bind(state_pub_addr_);
        service_addr_ = config.getValue("service_addr");
        std::cout<<"service_addr: "<<service_addr_ <<std::endl;
        //initialize franka robot
#if !LOCAL_TESTING
        try
        {
            gripper_ = std::make_shared<franka::Gripper>(gripper_ip_);
        }
        catch(const franka::NetworkException& e)
        {
            std::cerr << e.what() << '\n';
            this->stop();
        }
#endif
        // Register service handlers
        initializeService();
    }

    
    // Thread functions
    void statePublishThread() {
        zmq::socket_t pub_socket(context_, zmq::socket_type::pub);
        pub_socket.set(zmq::sockopt::sndhwm, 1000);
        pub_socket.set(zmq::sockopt::rcvtimeo, SOCKET_TIMEOUT_MS);
        pub_socket.connect(state_pub_addr_);

        while (is_running.load()) {
            franka::GripperState gripper_state = gripper_->readOnce();
            // Create FlatBuffers GripperState
            
            // Serialize and publish the gripper state
            flatbuffers::FlatBufferBuilder builder;
            auto msg = protocol::CreateFrankaGripperState(
                builder,
                gripper_state.time.toMSec(),
                gripper_state.width,
                gripper_state.max_width,
                gripper_state.is_grasped,
                gripper_state.temperature
            );
            zmq::message_t message(builder.GetSize());
            memcpy(message.data(), builder.GetBufferPointer(), builder.GetSize());
            try {
                pub_socket.send(message, zmq::send_flags::none);
            } catch (const zmq::error_t& e) {
                std::cerr << "[FrankaGripperProxy] ZMQ send error: " << e.what() << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / GRIPPER_PUB_RATE_HZ));
        }
        pub_socket.close();
    }

    bool setGripperWidth(const protocol::FrankaGripperCommand& command) {
        try {
            gripper_->grasp(command.width(), command.speed(), command.force());
            return true;
        } catch (const franka::Exception& e) {
            std::cerr << "[FrankaGripperProxy] Grasp failed: " << e.what() << std::endl;
            return false;
        }
    }

    std::string GetFrankaGripperStatePubPort() {
        return state_pub_addr_;
    }

private:
    // Configuration
    FrankaConfig gripper_config_;
    std::string gripper_ip_;
    std::string service_addr_;
    std::string state_pub_addr_;
    // Franka robot
    std::shared_ptr<franka::Gripper> gripper_;
    // ZMQ communication
    zmq::context_t context_;
    zmq::socket_t state_pub_socket_;//arm state publish socket
    
    // Threading
    std::thread state_pub_thread_;
    // Synchronization
    std::atomic<bool> is_running; // for threads
    

    // service registry
    void initializeService() {
        service_registry_.registerHandler("SetGripperWidth", this, &FrankaGripperProxy::setGripperWidth);
        service_registry_.registerHandler("GetFrankaGripperStatePubPort", this, &FrankaGripperProxy::GetFrankaGripperStatePubPort);
    };

    ServiceRegistry service_registry_;
    const std::string& getFrankaGripperStatePubPort();
    
    // TODO: put all the Constants to a config file
    static constexpr int STATE_PUB_RATE_HZ = 100;
    static constexpr int GRIPPER_PUB_RATE_HZ = 100;
    static constexpr int SOCKET_TIMEOUT_MS = 100;
    static constexpr int MAX_MESSAGE_SIZE = 4096;
};
