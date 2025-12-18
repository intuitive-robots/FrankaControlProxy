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
#include <zerolancom/zerolancom.hpp>
#include "utils/logger.hpp"

#include "utils/atomic_double_buffer.hpp"
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
        LOG_INFO("[{}] Control thread launched.", getModeName());
        setCommandSubscription();
        LOG_INFO("[{}] Command subscription thread launched.", getModeName());
    };

    void startRobot() {
        if (!robot_ || !model_) {
            LOG_ERROR("[{}] Robot or model not set.", getModeName());
            return;
        }
        robot_->automaticErrorRecovery();
        LOG_INFO("[{}] Robot control started.", getModeName());
        is_running_ = true;
    };

    virtual void stop() {
        is_running_ = false;
        if (control_thread_.joinable()) {
            LOG_INFO("[{}] Stopping control thread...", getModeName());
            control_thread_.join();
        }
        LOG_INFO("[{}] Stopped.", getModeName());
    };

    // Get the mode ID for this control mode
    virtual protocol::ModeID getModeID() const = 0; // Return the mode ID as an integer
    
    void init(std::shared_ptr<franka::Robot> robot, std::shared_ptr<franka::Model> model) {
        robot_ = std::move(robot);
        model_ = std::move(model);
    }

    void setCurrentStateBuffer(AtomicDoubleBuffer<franka::RobotState>& state_buffer) {
        current_state_buffer_ = &state_buffer;
    }

    virtual void setCommandSubscription() = 0;

    void updateRobotState(const franka::RobotState& new_state) {
        current_state_buffer_->write(new_state);
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
    AtomicDoubleBuffer<franka::RobotState>* current_state_buffer_ = nullptr;
    
    std::thread control_thread_;
    std::thread command_thread_;

    bool is_running_ = false;
    std::string command_sub_addr_;
    virtual void controlLoop() = 0;

    virtual void writeCommand(const protocol::ByteView& data) = 0;
    virtual void writeZeroCommand() = 0;
};
