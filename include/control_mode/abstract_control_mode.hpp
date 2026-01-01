#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <zerolancom/zerolancom.hpp>

#include "utils/atomic_double_buffer.hpp"
#include "utils/robot_utils.hpp"
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

    virtual ~AbstractControlMode() = default;

    void init(FrankaPanda& robot, FrankaModel& model, AtomicDoubleBuffer<franka::RobotState>& state_buffer) {
        robot_ = &robot;
        model_ = &model;
        state_buffer_ = &state_buffer;
        initController();;
    }

    virtual void initController() {};
    void startControl(AtomicDoubleBuffer<franka::RobotState>& state_buffer) {
        robot_->automaticErrorRecovery();
        zlc::info("[{}] Robot control started.", getModeName());
        is_running_ = true;
        control_thread_ = std::thread(&AbstractControlMode::controlLoop, this);
        zlc::info("[{}] Control thread launched.", getModeName());
    };
    void stopControl() {
        is_running_ = false;
        if (control_thread_.joinable()) {
            zlc::info("[{}] Stopping control thread...", getModeName());
            control_thread_.join();
        }
        zlc::info("[{}] Stopped.", getModeName());
    };
    const std::string getModeName() {
        return controller_name;
    };

protected:
    // Protected constructor to prevent direct instantiation
    AbstractControlMode() = default;
    // Protected setup function for derived classes
    FrankaPanda* robot_;
    FrankaModel* model_;
    AtomicDoubleBuffer<franka::RobotState>* state_buffer_;

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

    virtual void controlLoop() {};
    std::thread control_thread_;
    
};
