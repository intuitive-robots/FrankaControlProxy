#ifndef ABSTRACT_CONTROL_MODE_H
#define ABSTRACT_CONTROL_MODE_H

#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/exception.h>
#include <memory>//for std::shared_ptr
#include <mutex>
#include <zmq.hpp>
#include <iostream>

#include "utils/AtomicDoubleBuffer.hpp"
#include "protocol/franka_arm_state.hpp"

//todo:reform and check the leadter state get and the is_running
class AbstractControlMode {
public:
    // Virtual destructor for proper cleanup in derived classes
    virtual ~AbstractControlMode() = default;
    // Pure virtual public functions
    //virtual void initialize(const RobotState& initial_state);
    virtual void start() = 0;
    virtual void stop() = 0;
    // Get the mode ID for this control mode
    virtual int getModeID() const = 0; // Return the mode ID as an integer
    void setRobot(std::shared_ptr<franka::Robot> robot) {
        robot_ = std::move(robot);
    }
    void setModel(std::shared_ptr<franka::Model> model) {
        model_ = std::move(model);
    }
    
    // get current state of robot
    void updateRobotState(const franka::RobotState& new_state) {
        current_state_.write(new_state);
    }


protected:
    // Protected constructor to prevent direct instantiation
    AbstractControlMode() = default;
    // Protected setup function for derived classes
    std::shared_ptr<franka::Robot> robot_;
    std::shared_ptr<franka::Model> model_;
    AtomicDoubleBuffer<franka::RobotState> current_state_;
    
    bool is_running_ = false;
    
    void startCommandSubscriptionThread(const std::string& address) {
        zmq::socket_t sub_socket_(zmq::context_t(1), ZMQ_SUB);
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

    virtual void writeCommand(std::vector<uint8_t> data) {
        // Implementation for writing commands to the robot
    };
};
#endif // ABSTRACT_CONTROL_MODE_H