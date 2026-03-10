#include "robots/panda_gripper.hpp"

PandaGripper::PandaGripper(const std::string& config_path)
    : is_running_(false),
      command_(AtomicDoubleBuffer<GraspCommand>(GraspCommand{})),
      config_(config_path)
{
    gripper_ = std::make_shared<franka::Gripper>(config_.gripper_ip);
    gripper_->homing();
    // Initialize the command to the current width so the gripper doesn't immediately
    // try to close by default on startup.
    const franka::GripperState gs0 = gripper_->readOnce();
    command_.write(GraspCommand{static_cast<float>(gs0.width), 0.1});
    is_running_ = true;
    zlc::info("Gripper running flag set to {}", is_running_.load());
    state_pub_thread_ = std::thread(&PandaGripper::statePubThread, this);
    const std::string topic_name = fmt::format("{}/{}", config_.name, config_.command_topic);
    zlc::registerSubscriberHandler(topic_name, &PandaGripper::updateCommand, this);
    zlc::registerServiceHandler(fmt::format("{}/start_gripper_control", config_.name),
                                &PandaGripper::startControl, this);
    zlc::registerServiceHandler(fmt::format("{}/stop_gripper_control", config_.name),
                                &PandaGripper::stopControl, this);
}

PandaGripper::~PandaGripper()
{
    stop();
}

void PandaGripper::stop()
{
    zlc::info("Stopping PandaGripper...");
    is_running_ = false;
    if (state_pub_thread_.joinable())
        state_pub_thread_.join();
    if (control_thread_.joinable())
        control_thread_.join();
    gripper_.reset();
    zlc::info("PandaGripper stopped successfully.");
}

void PandaGripper::controlLoopThread()
{
    while (is_running_)
    {
        GraspCommand cmd = command_.read();
        try
        {
            gripper_->move(cmd.width, cmd.speed);
        }
        catch (const std::exception& e)
        {
            zlc::error("PandaGripper control loop exception: {}", e.what());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void PandaGripper::statePubThread()
{
    zlc::info("PandaGripper state publishing thread started.");
    const std::string topic_name = fmt::format("{}/{}", config_.name, config_.state_topic);
    zlc::Publisher<GripperStateMsg> state_publisher(topic_name);
    while (is_running_)
    {
        franka::GripperState gs = gripper_->readOnce();
        GripperStateMsg gsm{gs.width, gs.max_width, gs.is_grasped, gs.temperature,
                            gs.time.toMSec()};
        state_publisher.publish(gsm);
        std::this_thread::sleep_until(std::chrono::steady_clock::now() +
                                      std::chrono::milliseconds(1000 / GRIPPER_PUB_RATE_HZ));
    }
}

zlc::Empty PandaGripper::startControl(const zlc::Empty&)
{
    if (control_thread_.joinable())
    {
        zlc::warn("PandaGripper control loop thread is already running.");
        return zlc::Empty{};
    }
    is_running_ = true;
    control_thread_ = std::thread(&PandaGripper::controlLoopThread, this);
    zlc::info("PandaGripper control loop thread started.");
    return zlc::Empty{};
}

zlc::Empty PandaGripper::stopControl(const zlc::Empty&)
{
    if (control_thread_.joinable())
    {
        control_thread_.join();
        zlc::info("PandaGripper control loop thread stopped.");
    }
    return zlc::Empty{};
}

void PandaGripper::updateCommand(const GraspCommand& cmd)
{
    command_.write(cmd);
}
