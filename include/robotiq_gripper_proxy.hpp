#pragma once
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <memory>
#include <chrono>

#include <zerolancom/zerolancom.hpp>
#include "control_mode/abstract_control_mode.hpp"
#include "utils/atomic_double_buffer.hpp"
#include "robotiq/robotiq_gripper_interface.h"

// if public to often maybe block

struct RobotiqGripperStateMsg {
    float commanded_position;
    float commanded_speed;
    float commanded_force;
    float position;
    float current;// how powerful the gripper is closing
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
    bool blocking;   // Wait for completion if true

    RobotiqGraspCommand() : position(0.0f), speed(0.1f), force(0.1f), blocking(false) {}//default values
    RobotiqGraspCommand(float p, float s, float f, bool b = false)
        : position(p), speed(s), force(f), blocking(b) {}

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




class RobotiqGripperProxy {

public:
    // Constructor & Destructor
    explicit RobotiqGripperProxy(const std::string& config_path):
        is_running(false),
        command_(AtomicDoubleBuffer<RobotiqGraspCommand>(RobotiqGraspCommand{})),
        config_(config_path)
    {
        gripper_ = std::make_shared<robotiq::RobotiqGripperInterface>();
        bool connected = gripper_->connect(config_.port, config_.baud, config_.scale_alpha,
                                           config_.scale_beta);
        if (!connected) {
            zlc::error("RobotiqGripperProxy failed to connect on port {}", config_.port);
        } else {
            gripper_->set_timeout(config_.receive_timeout_ms);
            if (!gripper_->is_activated()) {
                zlc::info("RobotiqGripperProxy activating gripper...");
                gripper_->activate(true);
            }
        }
        command_.write(RobotiqGraspCommand{}); // initialize command buffer
        bool reset_success = gripper_->reset(true);
        zlc::info("RobotiqGripperProxy reset result: {}", reset_success);
        is_running = true;
        zlc::info("RobotiqGripper proxy running flag set to {}", is_running.load());
        zlc::registerSubscriberHandler(config_.command_topic, &RobotiqGripperProxy::updateCommand, this);
        command_.write(RobotiqGraspCommand{}); // initialize command buffer
        state_pub_thread_ = std::thread(&RobotiqGripperProxy::statePubThread, this);
        control_thread_ = std::thread(&RobotiqGripperProxy::controlLoopThread, this);
        robotiq::GripperFeedback gs = gripper_->get_feedback();
        zlc::info("RobotiqFeedback after init: Pos {:.2f}, CmdPos {}, Curr {:.2f}",
                   gs.position, gs.raw_commanded_position, gs.current);

    };
    ~RobotiqGripperProxy() {
        stop();
    };


    void stop() {
        zlc::info("Stopping RobotiqGripperProxy...");
        is_running = false;
        if (state_pub_thread_.joinable()) state_pub_thread_.join();
        if (control_thread_.joinable()) control_thread_.join();
        gripper_->reset(true);
        zlc::info("RobotiqGripperProxy stopped successfully.");
    };

private:
    bool commandChanged(const RobotiqGraspCommand& a, const RobotiqGraspCommand& b) const {
        return a.position != b.position || a.speed != b.speed || a.force != b.force ||
               a.blocking != b.blocking;
    }


    // Robotiq gripper
    std::shared_ptr<robotiq::RobotiqGripperInterface> gripper_;

    // Threading
    std::thread state_pub_thread_;
    std::thread control_thread_;

    // Threading Tasks
    void controlLoopThread() {
        RobotiqGraspCommand last_cmd = command_.read();
        while (is_running) {
            RobotiqGraspCommand cmd = command_.read();
            // zlc::info("RobotiqGripperProxy control loop read command: Pos {:.2f}, Speed {:.2f}, Force {:.2f}, Blocking {}",
            //           cmd.position, cmd.speed, cmd.force, cmd.blocking);
            if (gripper_) {
                try {
                    // zlc::info("RobotiqGripperProxy executing command: Pos {:.2f}, Speed {:.2f}, Force {:.2f}, Blocking {}",
                    //           cmd.position, cmd.speed, cmd.force, cmd.blocking);
                    if (commandChanged(cmd, last_cmd)) {
                        zlc::info("RobotiqGripperProxy executing new command: Pos {:.2f}, Speed {:.2f}, Force {:.2f}, Blocking {}",
                                  cmd.position, cmd.speed, cmd.force, cmd.blocking);
                        gripper_->set_gripper_position(cmd.position, cmd.speed, cmd.force, cmd.blocking);//or use set and read？
                        last_cmd = cmd;
                    }
                    // else {
                         // zlc::info("RobotiqGripperProxy command unchanged.");
                    // }
                }
                catch (const std::exception& e) {
                    zlc::error("RobotiqGripperProxy control loop exception: {}", e.what());
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    };

    void statePubThread() {
        zlc::info("RobotiqGripperProxy state publishing thread started.");
        const std::string topic_name = fmt::format("{}/{}", config_.name, config_.state_topic);
        zlc::Publisher<RobotiqGripperStateMsg> state_publisher(topic_name);
        while (is_running) {
            if (gripper_) {
                robotiq::GripperFeedback gs = gripper_->get_feedback();
                RobotiqGraspCommand last_cmd = command_.read();
                RobotiqGripperStateMsg gsm{
                    gs.commanded_position,
                    last_cmd.speed,
                    last_cmd.force,
                    gs.position,
                    gs.current,
                    gs.raw_commanded_position,
                    gs.raw_position
                };
                state_publisher.publish(gsm);
            }
            std::this_thread::sleep_until(
                std::chrono::steady_clock::now()
                    + std::chrono::milliseconds(1000 / config_.gripper_pub_rate_hz)
            );
        }
    };

    void updateCommand(const RobotiqGraspCommand& cmd) {
        command_.write(cmd);
        zlc::info("RobotiqGripperProxy received new command: Pos {:.2f}, Speed {:.2f}, Force {:.2f}, Blocking {}",
                  cmd.position, cmd.speed, cmd.force, cmd.blocking);
    };

    // Synchronization
    std::atomic<bool> is_running; // for threads

    AtomicDoubleBuffer<RobotiqGraspCommand> command_;

    RobotiqGripperConfig config_;
};
