#pragma once
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <memory>

#include <franka/gripper.h>
#include <franka/robot_state.h>
#include <zerolancom/zerolancom.hpp>
#include "control_mode/abstract_control_mode.hpp"
#include "utils/atomic_double_buffer.hpp"

struct GripperStateMsg {
    double width;
    double max_width;
    bool is_grasped;
    uint16_t temperature;
    uint64_t time;

    MSGPACK_DEFINE_MAP(width, max_width, is_grasped, temperature, time)
};

struct GraspCommand
{
    double width; // Desired width of the gripper in meters
    double speed; // Speed at which to close the gripper in meters per second

    GraspCommand() : width(0.0f), speed(0.0f) {}
    GraspCommand(float w, float s) : width(w), speed(s) {}

    MSGPACK_DEFINE_MAP(width, speed)
};

struct PandaGripperConfig
{
    // communication
    std::string name;
    std::string gripper_ip;

    std::string command_topic;
    std::string state_topic;
    bool enable_control{true};

    PandaGripperConfig(const std::string& gripper_config_path)
    {
        fromFile(gripper_config_path);
    }

    void fromFile(const std::string& gripper_config_path)
    {
        ConfigFileReader reader(gripper_config_path);
        name = reader.getValue<std::string>("name");
        gripper_ip = reader.getValue<std::string>("gripper_ip");
        command_topic = reader.getValue<std::string>("command_topic");
        state_topic = reader.getValue<std::string>("state_topic");
        enable_control = reader.getValue<bool>("enable_control", true);
    }
};

class PandaGripper {
public:
    // Constructor & Destructor
    explicit PandaGripper(const std::string& config_path);
    ~PandaGripper();

    void stop();

private:
    // Franka gripper
    std::shared_ptr<franka::Gripper> gripper_;

    // Threading
    std::thread state_pub_thread_;
    std::thread control_thread_;

    // Threading Tasks
    void controlLoopThread();
    void statePubThread();
    void updateCommand(const GraspCommand& cmd);
    zlc::Empty startControl(const zlc::Empty&);
    zlc::Empty stopControl(const zlc::Empty&);
    // Synchronization
    std::atomic<bool> is_running_;

    AtomicDoubleBuffer<GraspCommand> command_;

    PandaGripperConfig config_;

    static constexpr int GRIPPER_PUB_RATE_HZ = 100;
};
