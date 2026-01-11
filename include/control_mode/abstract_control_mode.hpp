#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <zerolancom/zerolancom.hpp>

#include "utils/atomic_double_buffer.hpp"
#include "utils/config_file_reader.hpp"
#include "utils/robot_model.hpp"
#include "utils/robot_utils.hpp"

struct ControllerConfig
{
    // communication
    std::string controller_name;
    std::string command_topic;
    ControllerConfig() = default;

    virtual void fromFile(const std::string& controller_config_path)
    {
        throw std::runtime_error("fromFile() not implemented");
    };
    void readBaseConfig(const ConfigFileReader& reader)
    {
        controller_name = reader.getValue<std::string>("name");
        command_topic = reader.getValue<std::string>("command_topic");
    }
};

class AbstractControlMode
{
  public:
    virtual ~AbstractControlMode() = default;

    virtual void initController(FrankaPanda& robot, PandaPinocchioModel& model,
                                AtomicDoubleBuffer<franka::RobotState>& state_buffer)
    {
        robot_ = &robot;
        model_ = &model;
        state_buffer_ = &state_buffer;
    };
    void startControl()
    {
        robot_->automaticErrorRecovery();
        zlc::info("[{}] Robot control started.", getModeName());
        is_running_ = true;
        control_thread_ = std::thread(&AbstractControlMode::controlTask, this);
        zlc::info("[{}] Control thread launched.", getModeName());
    };

    void stopControl()
    {
        is_running_ = false;
        if (control_thread_.joinable())
        {
            zlc::info("[{}] Stopping control thread...", getModeName());
            control_thread_.join();
        }
        zlc::info("[{}] Stopped.", getModeName());
    };
    const std::string getModeName()
    {
        return controller_name;
    };

    void controlTask()
    {
        zlc::info("[{}] Control thread started.", getModeName());
        auto control_callback = [this](const franka::RobotState& state,
                                       franka::Duration duration) -> franka::Torques
        { return this->controlLoop(state, duration); };
        while (is_running_)
        {
            try
            {
                robot_->control(control_callback);
            }
            catch (const std::exception& ex)
            {
                zlc::error("[CartesianVelocityMode] Robot is unable to be controlled: {}",
                           ex.what());
                break;
            }
            bool recovered = tryRecovery();
            if (!recovered)
            {
                zlc::error(
                    "[CartesianVelocityMode] Unable to recover robot. Exiting control loop.");
                break;
            }
        }
        zlc::info("[{}] Control thread ended.", getModeName());
    }

  protected:
    AbstractControlMode() = default;
    FrankaPanda* robot_;
    PandaPinocchioModel* model_;
    AtomicDoubleBuffer<franka::RobotState>* state_buffer_;

    std::string controller_name;

    bool is_running_ = false;

    bool tryRecovery(int max_attempts = 3)
    {
        for (size_t i = 0; i < max_attempts; i++)
        {
            try
            {
                robot_->automaticErrorRecovery();
                zlc::info("[{}] Recovery successful.", getModeName());
                return true;
            }
            catch (const franka::Exception& e)
            {
                zlc::error("[{}] Recovery failed: {}", getModeName(), e.what());
                return false;
            }
        }
        return false;
    };

    virtual franka::Torques controlLoop(const franka::RobotState& robot_state,
                                        franka::Duration duration) = 0;
    std::thread control_thread_;
private:
  void checkStateLimits()
  {
    // TODO: implement state limits checking
  }

  void postprocessTorques(franka::Torques& torques)
  {
    // TODO: implement torque post-processing
  }

};
