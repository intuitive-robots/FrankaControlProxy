#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <zerolancom/zerolancom.hpp>

#include "control_mode/abstract_control_mode.hpp"
#include "robotiq/robotiq_gripper_interface.h"
#include "utils/atomic_double_buffer.hpp"

struct RobotiqGripperStateMsg
{
    float commanded_position;
    float commanded_speed;
    float commanded_force;
    float position;
    float current; // how powerful the gripper is closing
    uint8_t raw_commanded_position;
    uint8_t raw_position;

    MSGPACK_DEFINE_MAP(commanded_position, commanded_speed, commanded_force, position, current,
                       raw_commanded_position, raw_position);
};

struct RobotiqGraspCommand
{
    float position; // Desired position, scaled by config scale_alpha/scale_beta
    float speed;    // Desired speed, scaled by config scale_alpha/scale_beta
    float force;    // Desired force, scaled by config scale_alpha/scale_beta
    bool blocking;  // Wait for completion if true

    RobotiqGraspCommand() : position(0.0f), speed(0.1f), force(0.1f), blocking(false) {}
    RobotiqGraspCommand(float p, float s, float f, bool b = false)
        : position(p), speed(s), force(f), blocking(b)
    {
    }

    MSGPACK_DEFINE_MAP(position, speed, force, blocking)
};

struct RobotiqGripperConfig
{
    // communication
    std::string name;
    std::string port;
    std::size_t baud{robotiq::DEFAULT_BAUD};

    std::string command_topic;
    std::string state_topic;
    double scale_alpha{robotiq::DEFAULT_SCALE_ALPHA};
    double scale_beta{robotiq::DEFAULT_SCALE_BETA};
    std::size_t receive_timeout_ms{robotiq::DEFAULT_RECEIVE_TIMEOUT_MS};
    int gripper_pub_rate_hz{100};

    RobotiqGripperConfig(const std::string& gripper_config_path)
    {
        fromFile(gripper_config_path);
    }

    void fromFile(const std::string& gripper_config_path)
    {
        ConfigFileReader reader(gripper_config_path);
        name = reader.getValue<std::string>("name", "RobotiqGripper");
        port = reader.getValue<std::string>("gripper_port", robotiq::DEFAULT_PORT);
        baud = reader.getValue<std::size_t>("gripper_baud", robotiq::DEFAULT_BAUD);
        command_topic = reader.getValue<std::string>("command_topic", "robotiq_gripper_command");
        state_topic = reader.getValue<std::string>("state_topic", "robotiq_gripper_state");
        scale_alpha = reader.getValue<double>("scale_alpha", robotiq::DEFAULT_SCALE_ALPHA);
        scale_beta = reader.getValue<double>("scale_beta", robotiq::DEFAULT_SCALE_BETA);
        receive_timeout_ms =
            reader.getValue<std::size_t>("receive_timeout_ms", robotiq::DEFAULT_RECEIVE_TIMEOUT_MS);
        gripper_pub_rate_hz = reader.getValue<int>("gripper_pub_rate_hz", 100);
    }
};

class RobotiqGripper
{
  public:
    // Constructor & Destructor
    explicit RobotiqGripper(const std::string& config_path);
    ~RobotiqGripper();

    void stop();

  private:
    bool commandChanged(const RobotiqGraspCommand& a, const RobotiqGraspCommand& b) const;

    // Robotiq gripper
    std::shared_ptr<robotiq::RobotiqGripperInterface> gripper_;

    // Threading
    std::thread state_pub_thread_;
    std::thread control_thread_;

    // Threading Tasks
    void controlLoopThread();
    void statePubThread();
    void updateCommand(const RobotiqGraspCommand& cmd);

    // Synchronization
    std::atomic<bool> is_running_;

    AtomicDoubleBuffer<RobotiqGraspCommand> command_;

    RobotiqGripperConfig config_;
};
