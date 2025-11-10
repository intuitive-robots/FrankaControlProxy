#pragma once
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/exception.h>
#include <memory>//for std::shared_ptr
#include <mutex>
#include <zmq.hpp>
#include <iostream>
#include <thread>
#include <zmq.hpp>

#include "utils/atomic_double_buffer.hpp"
#include "protocol/mode_id.hpp"

//todo:reform and check the leadter state get and the is_running
class AbstractControlMode {
public:
    // Virtual destructor for proper cleanup in derived classes
    virtual ~AbstractControlMode() = default;
    // Pure virtual public functions
    //virtual void initialize(const RobotState& initial_state);
    void start() {
        if (!robot_ || !model_) {
            std::cerr << "[CartesianVelocityMode] Robot or model not set.\n";
            return;
        }
        std::cout << "[CartesianVelocityMode] Started.\n";
        is_running_ = true;
        robot_->automaticErrorRecovery();
        std::thread control_thread(&AbstractControlMode::controlLoop, this);
        control_thread.detach();
        std::thread sub_thread(&AbstractControlMode::commandSubscriptionLoop, this, "tcp://localhost:5556");
        sub_thread.detach();
    };

    virtual void stop() {
        is_running_ = false;
        std::cout << "[" << getModeName() << "] Stopped.\n";
    };
    // Get the mode ID for this control mode
    virtual const std::string& getModeName() const = 0; // Return the mode ID as an integer
    void setRobot(std::shared_ptr<franka::Robot> robot) {
        robot_ = std::move(robot);
    }
    void setModel(std::shared_ptr<franka::Model> model) {
        model_ = std::move(model);
    }
    
    // get current state of robot
    void updateRobotState(const franka::RobotState& new_state) {
        current_state_->write(new_state);
    }

    protocol::ModeID getModeID() const {
        return protocol::fromString(getModeName());
    }

private:
    std::shared_ptr<zmq::context_t> context_;

protected:
    // Protected constructor to prevent direct instantiation
    AbstractControlMode() = default;
    // Protected setup function for derived classes
    std::shared_ptr<franka::Robot> robot_;
    std::shared_ptr<franka::Model> model_;
    std::shared_ptr<AtomicDoubleBuffer<franka::RobotState>> current_state_;
    
    bool is_running_ = false;

    virtual void controlLoop() = 0;

    void commandSubscriptionLoop(const std::string& address) {
        assert(context_ != nullptr && "ZMQ context must be initialized before starting subscription loop.");
        zmq::socket_t sub_socket_(*context_, ZMQ_SUB);
        sub_socket_.set(zmq::sockopt::rcvtimeo, 1000); // 1 second timeout
        sub_socket_.connect(address);
        sub_socket_.set(zmq::sockopt::subscribe, ""); // Subscribe to all messages
        while (is_running_) {
            try {
                zmq::message_t message; 
                auto result = sub_socket_.recv(message, zmq::recv_flags::none);
                if (!result) {
                    std::cerr << "[FrankaProxy] Failed to receive state message." << std::endl;
                    continue; // Skip this iteration if no message received
                }
                std::vector<uint8_t> data(static_cast<uint8_t*>(message.data()), static_cast<uint8_t*>(message.data()) + message.size());
                writeCommand(data);
            } catch (const zmq::error_t& e) {
                std::cerr << "[FrankaProxy] ZMQ recv error: " << e.what() << std::endl;
                break;
            }
        }
    };

    virtual void writeCommand(const std::vector<uint8_t>& data) = 0;
};
