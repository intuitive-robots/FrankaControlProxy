#pragma once
#include <zmq.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <memory>

#include <franka/gripper.h>
#include <franka/robot_state.h>
#include "control_mode/abstract_control_mode.hpp"
#include "utils/atomic_double_buffer.hpp"
#include "utils/service_registry.hpp" 
#include "utils/franka_config.hpp"
#include "protocol/codec.hpp"
#include "protocol/grasp_command.hpp"

enum class FrankaGripperFlag {
    STOP = 0,
    STOPPING = 1,
    CLOSING = 2,
    OPENING = 3,
};


class FrankaGripperProxy {

public:
    // Constructor & Destructor
    explicit FrankaGripperProxy(const std::string& config_path):
        is_running(false),
        is_on_control_mode(false),
        gripper_flag(FrankaGripperFlag::STOP),
        current_state_(AtomicDoubleBuffer<franka::GripperState>(franka::GripperState{})),
        command_(AtomicDoubleBuffer<protocol::GraspCommand>(protocol::GraspCommand{}))
    {
        FrankaConfig config(config_path);
        gripper_ip_ = config.getValue("gripper_ip");
        state_pub_addr_ = config.getValue("gripper_state_pub_addr");
        service_registry_.bindSocket(config.getValue("gripper_service_addr"));
        //initialize franka gripper
# if !LOCAL_TESTING
        gripper_ = std::make_shared<franka::Gripper>(gripper_ip_);
        gripper_->homing();
        current_state_.write(gripper_->readOnce());
        command_.write(protocol::GraspCommand{current_state_.read().width, 0.01f, 20.0f});
# else
        current_state_.write(franka::GripperState{0.0f, 0.0f, false, false, franka::Duration{}});
        command_.write(protocol::GraspCommand{0.0f, 0.01f, 20.0f});
# endif
        initializeService();
    };
    ~FrankaGripperProxy() {
        stop();
    };

    // Core server operations
    void start() {
        is_running = true;
        is_moving = false;
        std::cout << is_running <<"gripper control"<< std::endl;
        // state_pub_thread_ = std::thread(&FrankaGripperProxy::statePublishThread, this);
        service_registry_.start();
        command_.write(protocol::GraspCommand{(float)current_state_.read().width, 20.0f, 20.0f});
        state_pub_thread_ = std::thread(&FrankaGripperProxy::statePubThread, this);
        check_thread_ = std::thread(&FrankaGripperProxy::checkWidthThread, this);
        control_thread_ = std::thread(&FrankaGripperProxy::controlLoopThread, this);
    };

    void stop() {
        std::cout << "[INFO] Stopping FrankaGripperProxy..." << std::endl;
        is_running = false;
        is_on_control_mode = false;
        if (state_pub_thread_.joinable()) state_pub_thread_.join();
        if (control_thread_.joinable()) control_thread_.join();
        if (check_thread_.joinable()) check_thread_.join();
        if (command_sub_thread_.joinable()) command_sub_thread_.join();

        // wait for closing
        service_registry_.stop();
        gripper_.reset();
        std::cout << "[INFO] FrankaGripperProxy stopped successfully." << std::endl;
    };

private:
    // Initialization
    void initializeService() {
        // Register service handlers
        service_registry_.registerHandler("START_FRANKA_GRIPPER_CONTROL", this, &FrankaGripperProxy::startFrankaGripperControl);
        service_registry_.registerHandler("GET_FRANKA_GRIPPER_STATE_PUB_PORT", this, &FrankaGripperProxy::getFrankaGripperStatePubPort);
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
    std::thread control_thread_;
    std::thread check_thread_;
    std::thread command_sub_thread_;
    
    // Threading Tasks
    void controlLoopThread() {
        while (is_running) {
            float current_width = current_state_.read().width;
            float target_width = command_.read().width;
            if (gripper_flag.load() == FrankaGripperFlag::STOPPING) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            try
            {
                if (current_width > target_width + 0.01) {
                    std::cout << "[Gripper Close] Closing gripper to target width: " << target_width << std::endl;
                    gripper_flag.store(FrankaGripperFlag::CLOSING);
                    gripper_->grasp(target_width, command_.read().speed, 60.0);
                } else if (current_width < target_width - 0.01) {
                    std::cout << "[Gripper Open] Opening gripper to target width: " << target_width << std::endl;
                    gripper_flag.store(FrankaGripperFlag::OPENING);
                    gripper_->move(target_width, command_.read().speed);
                }
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            gripper_flag.store(FrankaGripperFlag::STOP);
        }
    };

    void statePubThread() {
        zmq::socket_t pub_socket_(ZmqContext::instance(), zmq::socket_type::pub);
        pub_socket_.bind(state_pub_addr_);
        while (is_running) {
#if !LOCAL_TESTING
            franka::GripperState gs = gripper_->readOnce();
            current_state_.write(gs);
            pub_socket_.send(zmq::const_buffer(protocol::encode(gs).data(),
                                                protocol::encode(gs).size()),
                                zmq::send_flags::none);
#endif
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / GRIPPER_PUB_RATE_HZ));
        }
        try {
            state_pub_socket_.close();
        } catch (const zmq::error_t& e) {
            std::cerr << "[ZMQ ERROR] " << e.what() << "\n";
        }
    };

    void checkWidthThread() {
        while (is_running) {
#if !LOCAL_TESTING
            float current_width = current_state_.read().width;
            float target_width = command_.read().width;
            if (
                gripper_flag.load() == FrankaGripperFlag::STOPPING
            || (gripper_flag.load() == FrankaGripperFlag::CLOSING && current_width >= target_width + 0.01)
            || (gripper_flag.load() == FrankaGripperFlag::OPENING && current_width <= target_width - 0.01)
            || (gripper_flag.load() == FrankaGripperFlag::STOP && std::abs(current_width - target_width) <= 0.01)
            ) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            try
            {
                std::cout << "[Gripper Stop] Gripper reached target width: " << target_width << std::endl;
                gripper_flag.store(FrankaGripperFlag::STOP);
                gripper_->stop();
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
#endif
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    };

    void commandSubThread(const std::string& command_sub_addr) {
        is_on_control_mode = true;
        zmq::socket_t sub_socket_(ZmqContext::instance(), ZMQ_SUB);
        sub_socket_.set(zmq::sockopt::rcvtimeo, 500); // 0.5 second timeout
        sub_socket_.set(zmq::sockopt::subscribe, ""); // Subscribe to all messages
        sub_socket_.connect(command_sub_addr);
        while (is_running && is_on_control_mode) {
            sub_socket_.set(zmq::sockopt::subscribe, ""); // Subscribe to all messages
            zmq::message_t message;
            if (!sub_socket_.recv(message, zmq::recv_flags::none)) {
                continue;
            }
            protocol::ByteView data{
                static_cast<const uint8_t*>(message.data()),
                message.size()
            };
            command_.write(protocol::decode<protocol::GraspCommand>(data));
            // std::this_thread::sleep_for(std::chrono::milliseconds(10));
            // std::cout << "[FrankaGripperProxy] Received new gripper command: width="
            //           << command_.read().width << ", speed=" << command_.read().speed
            //           << ", force=" << command_.read().force << std::endl;
        }
    };


    // Synchronization
    std::atomic<bool> is_running; // for threads
    std::atomic<bool> is_moving;
    std::atomic<bool> is_on_control_mode;
    std::atomic<FrankaGripperFlag> gripper_flag;
    
    AtomicDoubleBuffer<franka::GripperState> current_state_;
    AtomicDoubleBuffer<protocol::GraspCommand> command_;

    // service registry
    ServiceRegistry service_registry_;
    void startFrankaGripperControl(const std::string& command_sub_addr) {
        if (is_on_control_mode) {
            std::cout << "[FrankaGripperProxy] Already in control mode, now quit previous control mode." << std::endl;
            is_on_control_mode = false;
            if (command_sub_thread_.joinable()) command_sub_thread_.join();
        }
        command_sub_thread_ = std::thread(&FrankaGripperProxy::commandSubThread, this, command_sub_addr);
    };
    const std::string& getFrankaGripperStatePubPort() {
        return state_pub_addr_;
    };

    // TODO: put all the Constants to a config file
    static constexpr int STATE_PUB_RATE_HZ = 100;
    static constexpr int GRIPPER_PUB_RATE_HZ = 100;
    static constexpr int SOCKET_TIMEOUT_MS = 100;
    static constexpr int MAX_MESSAGE_SIZE = 4096;
};
