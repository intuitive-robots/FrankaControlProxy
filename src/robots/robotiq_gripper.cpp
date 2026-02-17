#include "robots/robotiq_gripper.hpp"

RobotiqGripper::RobotiqGripper(const std::string& config_path)
    : is_running_(false),
      command_(AtomicDoubleBuffer<RobotiqGraspCommand>(RobotiqGraspCommand{})),
      config_(config_path)
{
    gripper_ = std::make_shared<robotiq::RobotiqGripperInterface>();
    bool connected = gripper_->connect(config_.port, config_.baud, config_.scale_alpha,
                                       config_.scale_beta);
    if (!connected) {
        zlc::error("RobotiqGripper failed to connect on port {}", config_.port);
    } else {
        gripper_->set_timeout(config_.receive_timeout_ms);
        if (!gripper_->is_activated()) {
            zlc::info("RobotiqGripper activating gripper...");
            gripper_->activate(true);
        }
    }
    command_.write(RobotiqGraspCommand{}); // initialize command buffer
    bool reset_success = gripper_->reset(true);
    zlc::info("RobotiqGripper reset result: {}", reset_success);
    is_running_ = true;
    zlc::info("RobotiqGripper running flag set to {}", is_running_.load());
    zlc::registerSubscriberHandler(config_.command_topic, &RobotiqGripper::updateCommand, this);
    command_.write(RobotiqGraspCommand{}); // initialize command buffer
    state_pub_thread_ = std::thread(&RobotiqGripper::statePubThread, this);
    control_thread_ = std::thread(&RobotiqGripper::controlLoopThread, this);
    robotiq::GripperFeedback gs = gripper_->get_feedback();
    zlc::info("RobotiqFeedback after init: Pos {:.2f}, CmdPos {}, Curr {:.2f}",
               gs.position, gs.raw_commanded_position, gs.current);
}

RobotiqGripper::~RobotiqGripper()
{
    stop();
}

void RobotiqGripper::stop()
{
    zlc::info("Stopping RobotiqGripper...");
    is_running_ = false;
    if (state_pub_thread_.joinable()) state_pub_thread_.join();
    if (control_thread_.joinable()) control_thread_.join();
    gripper_->reset(true);
    zlc::info("RobotiqGripper stopped successfully.");
}

bool RobotiqGripper::commandChanged(const RobotiqGraspCommand& a, const RobotiqGraspCommand& b) const
{
    return a.position != b.position || a.speed != b.speed || a.force != b.force ||
           a.blocking != b.blocking;
}

void RobotiqGripper::controlLoopThread()
{
    RobotiqGraspCommand last_cmd = command_.read();
    while (is_running_) {
        RobotiqGraspCommand cmd = command_.read();
        if (gripper_) {
            try {
                if (commandChanged(cmd, last_cmd)) {
                    // zlc::info("RobotiqGripper executing new command: Pos {:.2f}, Speed {:.2f}, Force {:.2f}, Blocking {}",
                    //           cmd.position, cmd.speed, cmd.force, cmd.blocking);
                    gripper_->set_gripper_position(cmd.position, cmd.speed, cmd.force, cmd.blocking);
                    last_cmd = cmd;
                }
            }
            catch (const std::exception& e) {
                zlc::error("RobotiqGripper control loop exception: {}", e.what());
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void RobotiqGripper::statePubThread()
{
    zlc::info("RobotiqGripper state publishing thread started.");
    const std::string topic_name = fmt::format("{}/{}", config_.name, config_.state_topic);
    zlc::Publisher<RobotiqGripperStateMsg> state_publisher(topic_name);
    while (is_running_) {
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
}

void RobotiqGripper::updateCommand(const RobotiqGraspCommand& cmd)
{
    command_.write(cmd);
    // zlc::info("RobotiqGripper received new command: Pos {:.2f}, Speed {:.2f}, Force {:.2f}, Blocking {}",
    //           cmd.position, cmd.speed, cmd.force, cmd.blocking);
}
