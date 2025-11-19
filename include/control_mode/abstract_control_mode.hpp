#pragma once
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/exception.h>
#include <memory>
#include <mutex>
#include <zmq.hpp>
#include <thread>
#include <zmq.hpp>
#include <spdlog/spdlog.h>

#include "utils/atomic_double_buffer.hpp"
#include "utils/zmq_context.hpp"
#include "protocol/mode_id.hpp"
#include "protocol/codec.hpp"

//todo:reform and check the leadter state get and the is_running
class AbstractControlMode {
public:
    // Virtual destructor for proper cleanup in derived classes
    virtual ~AbstractControlMode() = default;
    // Pure virtual public functions
    //virtual void initialize(const RobotState& initial_state);
    virtual void start() {
        startRobot();
        control_thread_ = std::thread(&AbstractControlMode::controlLoop, this);
        spdlog::info("[{}] Control thread launched.", getModeName());
        command_thread_ = std::thread(&AbstractControlMode::commandSubscriptionLoop, this);
        spdlog::info("[{}] Command subscription thread launched.", getModeName());
    };

    void startRobot() {
#if !LOCAL_TESTING
        if (!robot_ || !model_) {
            spdlog::error("[{}] Robot or model not set.", getModeName());
            return;
        }
        robot_->automaticErrorRecovery();
#endif
        spdlog::info("[{}] Robot control started.", getModeName());
        is_running_ = true;
    };

    virtual void stop() {
        is_running_ = false;
        if (control_thread_.joinable()) {
            spdlog::info("[{}] Stopping control thread...", getModeName());
            control_thread_.join();
        }
        if (command_thread_.joinable()) {
            spdlog::info("[{}] Stopping command subscription thread...", getModeName());
            command_thread_.join();
        }
        spdlog::info("[{}] Stopped.", getModeName());
    };
    // Get the mode ID for this control mode
    virtual protocol::ModeID getModeID() const = 0; // Return the mode ID as an integer
    void setRobot(std::shared_ptr<franka::Robot> robot) {
        robot_ = std::move(robot);
    }
    void setModel(std::shared_ptr<franka::Model> model) {
        model_ = std::move(model);
    }
    void setCurrentStateBuffer(AtomicDoubleBuffer<franka::RobotState>* state_buffer) {
        current_state_ = state_buffer;
    }
    void setupCommandSubscription(const std::string& address) {
        command_sub_addr_ = address;
    }
    // get current state of robot
    void updateRobotState(const franka::RobotState& new_state) {
        current_state_->write(new_state);
    }

    const std::string getModeName() const {
        return protocol::toString(getModeID());
    }

protected:
    // Protected constructor to prevent direct instantiation
    AbstractControlMode() = default;
    // Protected setup function for derived classes
    std::shared_ptr<franka::Robot> robot_;
    std::shared_ptr<franka::Model> model_;
    AtomicDoubleBuffer<franka::RobotState>* current_state_ = nullptr;
    
    std::thread control_thread_;
    std::thread command_thread_;

    bool is_running_ = false;
    std::string command_sub_addr_;
    virtual void controlLoop() = 0;

    void commandSubscriptionLoop() {
        zmq::socket_t sub_socket_(ZmqContext::instance(), ZMQ_SUB);
        sub_socket_.set(zmq::sockopt::rcvtimeo, 200); // 100 ms timeout
        if (command_sub_addr_.empty()) {
            spdlog::warn("[{}] Command subscription address is empty. Exiting command subscription loop.", getModeName());
            return;
        }
        sub_socket_.connect(command_sub_addr_);
        sub_socket_.set(zmq::sockopt::subscribe, ""); // Subscribe to all messages
        while (is_running_) {
            try {
                zmq::message_t message;
                if (!sub_socket_.recv(message, zmq::recv_flags::none)) {
                    writeZeroCommand();
                    continue; // Skip this iteration if no message received
                }
                protocol::ByteView data{
                    static_cast<const uint8_t*>(message.data()),
                    message.size()
                };
                writeCommand(data);
            } catch (const zmq::error_t& e) {
                spdlog::error("[FrankaProxy] ZMQ recv error: {}", e.what());
                break;
            }
        }
        sub_socket_.close();
        spdlog::info("[{}] Command subscription loop exited.", getModeName());
    };

    virtual void writeCommand(const protocol::ByteView& data) = 0;
    virtual void writeZeroCommand() = 0;
};
