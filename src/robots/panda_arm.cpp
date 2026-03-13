#include "robots/panda_arm.hpp"

#include <algorithm>

static std::atomic<bool> running_flag{true}; // let ctrl-c stop the server
static void signalHandler(int signum)
{
    zlc::info("Caught signal {}, shutting down...", signum);
    running_flag = false;
}

PandaArm::PandaArm(const std::string& config_path)
    : is_running(false),
      current_state(AtomicDoubleBuffer<franka::RobotState>(franka::RobotState{})),
      config_(config_path)
{
    // Register service handlers
    initRobot();

    // Check if robot and model initialization was successful
    if (!robot_ || !model_)
    {
        zlc::error("Robot or model initialization failed. Proxy cannot continue.");
        is_running = false;
        return;
    }
    control_mode_factory_ = std::make_unique<ControlModeFactory>(config_.name);
    control_mode_factory_->registerControlModes(*robot_, *model_, current_state);
    setControlMode("Idle");
    initializeService();
}

void PandaArm::initializeService()
{
    std::string service_namespace = fmt::format("{}/set_franka_arm_control_mode", config_.name);
    zlc::registerServiceHandler(service_namespace, &PandaArm::setControlMode, this);
    service_namespace = fmt::format("{}/get_franka_arm_state", config_.name);
    zlc::registerServiceHandler(service_namespace, &PandaArm::getFrankaArmState, this);
    service_namespace = fmt::format("{}/get_franka_arm_control_mode", config_.name);
    zlc::registerServiceHandler(service_namespace, &PandaArm::getFrankaArmControlMode, this);
    service_namespace = fmt::format("{}/move_franka_arm_to_joint_position", config_.name);
    zlc::registerServiceHandler(service_namespace, &PandaArm::moveFrankaArmToJointPosition, this);
    service_namespace = fmt::format("{}/move_franka_arm_to_cartesian_position", config_.name);
}

PandaArm::~PandaArm()
{
    stop();
}

void PandaArm::initRobot()
{
    //initialize franka robot
    try
    {
        robot_ = std::make_unique<FrankaPanda>(config_.robot_ip);
        model_ = std::make_unique<PandaPinocchioModel>("./models/franka_emika_panda/panda_arm.urdf",
                                                       "panda_link8");
    }
    catch (const franka::NetworkException& e)
    {
        zlc::error("Franka Network Exception: {}", e.what());
        this->stop();
        return;
    }
    catch (const std::exception& e)
    {
        zlc::error("Exception during robot/model initialization: {}", e.what());
        this->stop();
        return;
    }
    catch (...)
    {
        zlc::error("Unknown exception during robot/model initialization");
        this->stop();
        return;
    }
    current_state.write(robot_->readOnce());
    state_pub_thread = std::thread(&PandaArm::statePublishThread, this);
    zlc::info("PandaArm started successfully.");
    is_running = true;
}

void PandaArm::stop()
{
    zlc::info("Stopping PandaArm...");
    is_running = false;
    if (current_control_mode_)
        current_control_mode_->stopControl();
    if (state_pub_thread.joinable())
        state_pub_thread.join();
    // wait for closing
    robot_.reset();
    model_.reset();
    current_control_mode_ = nullptr; // reset current control mode
    zlc::info("PandaArm stopped successfully.");
}

// Main loop for processing requests, ctrl-c to stop the server
void PandaArm::spin()
{
    std::signal(SIGINT, signalHandler); //  Catch Ctrl+C to stop the server
    zlc::info("Entering main spin loop. Press Ctrl+C to exit.");
    zlc::info("Current running flag: {}", running_flag.load());
    while (running_flag)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    stop();
    zlc::info("Shutdown complete.");
}

// publish threads
void PandaArm::statePublishThread()
{
    const std::string topic_name = fmt::format("{}/franka_arm_state", config_.name);
    zlc::Publisher<FrankaArmState> state_pub(topic_name);
    int dt = int(1000.0 / config_.arm_state_pub_rate_hz);
    while (is_running)
    {
        const franka::RobotState rs = current_state.read();
        FrankaArmState frs(rs);
        state_pub.publish(frs);
        std::this_thread::sleep_for(std::chrono::milliseconds(dt));
    }
}

zlc::Empty PandaArm::setControlMode(const std::string& mode)
{
    if (control_mode_factory_->getController(mode) == nullptr)
    {
        zlc::warn("Control mode '{}' not found!", mode);
        return zlc::empty;
    }
    if (current_control_mode_ != nullptr)
    {
        zlc::info("Stopping previous control mode...");
        current_control_mode_->stopControl();
    }
    zlc::info("Switching to control mode: {}", mode);
    current_control_mode_ = control_mode_factory_->getController(mode);
    current_control_mode_->startControl();
    return zlc::empty;
}

FrankaArmState PandaArm::getFrankaArmState(const zlc::Empty&)
{
    return FrankaArmState(current_state.read());
}

std::string PandaArm::getFrankaArmControlMode(const zlc::Empty&)
{
    if (!current_control_mode_)
    {
        zlc::warn("No active control mode");
        return "None";
    }
    return current_control_mode_->getModeName();
}

std::pair<std::string, std::vector<uint8_t>> PandaArm::moveFrankaArmToJointPosition(
    const std::vector<double>& target_q)
{
    if (!current_control_mode_)
    {
        return {std::string(protocol::FrankaResponseCode::FAIL), {}};
    }
    if (target_q.size() != 7)
    {
        zlc::warn("moveFrankaArmToJointPosition: invalid payload size {} (expected 7)",
                  target_q.size());
        return {std::string(protocol::FrankaResponseCode::INVALID_ARG), {}};
    }
    std::array<double, 7> target_q_array{};
    std::copy(target_q.begin(), target_q.end(), target_q_array.begin());
    // current_control_mode_->stopControl();
    const bool ok = current_control_mode_->moveToJointPosition(target_q_array);
    return {std::string(ok ? protocol::FrankaResponseCode::SUCCESS
                           : protocol::FrankaResponseCode::FAIL),
            {}};
}
