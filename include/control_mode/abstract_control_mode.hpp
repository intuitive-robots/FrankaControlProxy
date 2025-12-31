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

#include "utils/atomic_double_buffer.hpp"
#include "protocol/mode_id.hpp"
#include "protocol/codec.hpp"
#include "utils/config_file_reader.hpp"

struct ControllerConfig {
    // communication
    std::string controller_name;
    std::string command_topic;
    ControllerConfig() = delete;
    ControllerConfig(const std::string& controller_config_path) {
        fromFile(controller_config_path);
    }
    virtual void fromFile(const std::string& controller_config_path) {};
    void readBaseConfig(const ConfigFileReader& reader) {
        controller_name = reader.getValue<std::string>("name", "UnnamedController");
        command_topic = reader.getValue<std::string>("command_topic", "UNNAMED_CMD");
    }
};



class AbstractControlMode {
public:
    // Virtual destructor for proper cleanup in derived classes
    virtual ~AbstractControlMode() = default;
    // Pure virtual public functions
    //virtual void initialize(const RobotState& initial_state);
    // virtual void start() {
    //     startRobot();
    //     control_thread_ = std::thread(&AbstractControlMode::controlLoop, this);
    //     zlc::info("[{}] Control thread launched.", getModeName());
    //     zlc::info("[{}] Command subscription thread launched.", getModeName());
    // };

    void init(std::shared_ptr<franka::Robot> robot, std::shared_ptr<franka::Model> model) {
        robot_ = std::move(robot);
        model_ = std::move(model);
        initController();;
    }

    virtual void initController() {};
    virtual void startControl(AtomicDoubleBuffer<franka::RobotState>& state_buffer) = 0;
    virtual void stopControl() = 0;
    const std::string getModeName() {
        return controller_name;
    };

protected:
    // Protected constructor to prevent direct instantiation
    AbstractControlMode() = default;
    // Protected setup function for derived classes
    std::shared_ptr<franka::Robot> robot_;
    std::shared_ptr<franka::Model> model_;
    
    std::thread control_thread_;
    const std::string controller_name;

    bool is_running_ = false;

    bool tryRecovery(int max_attempts = 3) {
        for (size_t i = 0; i < max_attempts; i++)
        {
            try {
                robot_->automaticErrorRecovery();
                zlc::info("[{}] Recovery successful.", getModeName());
                return true;
            } catch (const franka::Exception& e) {
                zlc::error("[{}] Recovery failed: {}", getModeName(), e.what());
                return false;
            }
        }
        return false;
    };
    
private:
    
    void startRobot() {
        if (!robot_ || !model_) {
            zlc::error("[{}] Robot or model not set.", getModeName());
            return;
        }
        robot_->automaticErrorRecovery();
        zlc::info("[{}] Robot control started.", getModeName());
        is_running_ = true;
    };
    
    void stopControlThread() {
        is_running_ = false;
        if (control_thread_.joinable()) {
            zlc::info("[{}] Stopping control thread...", getModeName());
            control_thread_.join();
        }
        zlc::info("[{}] Stopped.", getModeName());
    };
    
};
