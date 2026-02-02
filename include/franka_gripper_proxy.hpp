#pragma once
#include <zmq.hpp>
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



struct FrankaGripperConfig
{
    // communication
    std::string name;
    std::string gripper_ip;

    std::string command_topic;
    std::string state_topic;
    bool enable_control{true};

    FrankaGripperConfig(const std::string& gripper_config_path)
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




class FrankaGripperProxy {

public:
    // Constructor & Destructor
    explicit FrankaGripperProxy(const std::string& config_path):
        is_running(false),
        command_(AtomicDoubleBuffer<GraspCommand>(GraspCommand{})),
        config_(config_path)
    {
        gripper_ = std::make_shared<franka::Gripper>(config_.gripper_ip);
        gripper_->homing();
        // Initialize the command to the current width so the gripper doesn't immediately
        // try to close by default on startup.
        const franka::GripperState gs0 = gripper_->readOnce();
        command_.write(GraspCommand{static_cast<float>(gs0.width), 0.1f});
        is_running = true;
        zlc::info("Gripper proxy running flag set to {}", is_running.load());
        state_pub_thread_ = std::thread(&FrankaGripperProxy::statePubThread, this);
        if (config_.enable_control)
        {
            zlc::registerSubscriberHandler(config_.command_topic, &FrankaGripperProxy::updateCommand, this);
            control_thread_ = std::thread(&FrankaGripperProxy::controlLoopThread, this);
            zlc::info("Gripper control enabled: subscribing to '{}'", config_.command_topic);
        }
        else
        {
            zlc::info("Gripper control disabled (monitor-only): publishing '{}' only", config_.state_topic);
        }

    };
    ~FrankaGripperProxy() {
        stop();
    };


    void stop() {
        zlc::info("Stopping FrankaGripperProxy...");
        is_running = false;
        if (state_pub_thread_.joinable()) state_pub_thread_.join();
        if (control_thread_.joinable()) control_thread_.join();
        gripper_.reset();
        zlc::info("FrankaGripperProxy stopped successfully.");
    };

private:

    // Franka robot
    std::shared_ptr<franka::Gripper> gripper_;

    // Threading
    std::thread state_pub_thread_;
    std::thread control_thread_;

    // Threading Tasks
    void controlLoopThread() {
        while (is_running) {
            GraspCommand cmd = command_.read();
            try {
                gripper_->move(cmd.width, cmd.speed);
            }
            catch (const std::exception& e) {
                zlc::error("FrankaGripperProxy control loop exception: {}", e.what());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    };

    void statePubThread() {
        zlc::info("FrankaGripperProxy state publishing thread started.");
        const std::string topic_name = fmt::format("{}/{}", config_.name, config_.state_topic);
        zlc::Publisher<GripperStateMsg> state_publisher(topic_name);
        while (is_running) {
            franka::GripperState gs = gripper_->readOnce();
            GripperStateMsg gsm{
                gs.width,
                gs.max_width,
                gs.is_grasped,
                gs.temperature,
                gs.time.toMSec()
            };
            state_publisher.publish(gsm);
            std::this_thread::sleep_until(
                std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / GRIPPER_PUB_RATE_HZ)
            );
        }
    };

    void updateCommand(const GraspCommand& cmd) {
        command_.write(cmd);
    };

    // Synchronization
    std::atomic<bool> is_running; // for threads

    AtomicDoubleBuffer<GraspCommand> command_;

    FrankaGripperConfig config_;

    // TODO: put all the Constants to a config file
    static constexpr int GRIPPER_PUB_RATE_HZ = 100;
};
