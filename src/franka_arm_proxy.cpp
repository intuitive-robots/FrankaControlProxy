
#include "franka_arm_proxy.hpp"

static std::atomic<bool> running_flag{true}; // let ctrl-c stop the server
static void signalHandler(int signum)
{
    zlc::info("Caught signal {}, shutting down...", signum);
    running_flag = false;
}
// namespace {
franka::RobotState createDefaultState(const FrankaArmConfig& cfg)
{
    franka::RobotState state{};
    std::array<double, 7> q{};
    const auto& q_src = cfg.arm_default_state_q.size() == 7
                            ? cfg.arm_default_state_q
                            : std::array<double, 7>{{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}};
    std::copy(q_src.begin(), q_src.begin() + 7, q.begin());
    state.q = q;

    std::array<double, 16> pose = cfg.arm_default_state_O_T_EE;
    if (pose == std::array<double, 16>{})
    {
        pose = {{1.0, 0.0, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.0, 0.0, 1.0}};
    }
    state.O_T_EE = pose;
    return state;
}
// }

FrankaArmProxy::FrankaArmProxy(const std::string& config_path)
    : is_running(false),
      config_(config_path),
      current_state(AtomicDoubleBuffer<franka::RobotState>(createDefaultState(config_)))
{
    // Register service handlers
    initRobot();
    ControlModeFactory::registerControlModes(control_modes_, *robot_, *model_, current_state);
    setControlMode("Idle");
    initializeService();
}

void FrankaArmProxy::initializeService()
{
    zlc::registerServiceHandler("SET_FRANKA_ARM_CONTROL_MODE", &FrankaArmProxy::setControlMode,
                                this);
    zlc::registerServiceHandler("GET_FRANKA_ARM_STATE", &FrankaArmProxy::getFrankaArmState, this);
    zlc::registerServiceHandler("GET_FRANKA_ARM_CONTROL_MODE",
                                &FrankaArmProxy::getFrankaArmControlMode, this);
    // zlc::registerServiceHandler("MOVE_FRANKA_ARM_TO_JOINT_POSITION", &FrankaArmProxy::moveFrankaArmToJointPosition, this);
    // zlc::registerServiceHandler("MOVE_FRANKA_ARM_TO_CARTESIAN_POSITION", &FrankaArmProxy::moveFrankaArmToCartesianPosition, this);
}

FrankaArmProxy::~FrankaArmProxy()
{
    stop();
}

void FrankaArmProxy::initRobot()
{
    //initialize franka robot
    try
    {
        robot_ = std::make_unique<FrankaPanda>(config_.robot_ip);
        model_ = nullptr;
    }
    catch (const franka::NetworkException& e)
    {
        zlc::error("{}", e.what());
        this->stop();
    }
    current_state.write(robot_->readOnce());
    state_pub_thread = std::thread(&FrankaArmProxy::statePublishThread, this);
    zlc::info("FrankaArmProxy started successfully.");
    is_running = true;
}

void FrankaArmProxy::stop()
{
    zlc::info("Stopping FrankaArmProxy...");
    is_running = false;
    if (current_control_mode_)
        current_control_mode_->stopControl();
    if (state_pub_thread.joinable())
        state_pub_thread.join();
    // wait for closing
    robot_.reset();
    model_.reset();
    current_control_mode_ = nullptr; // reset current control mode
    zlc::info("FrankaArmProxy stopped successfully.");
}

// Main loop for processing requests, ctrl-c to stop the server
void FrankaArmProxy::spin()
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
void FrankaArmProxy::statePublishThread()
{
    zlc::Publisher<FrankaArmState> state_pub("FRANKA_ARM_STATE_PUB");
    int dt = int(1000.0 / config_.arm_state_pub_rate_hz);
    while (is_running)
    {
        const franka::RobotState rs = current_state.read();
        FrankaArmState frs(rs);
        state_pub.publish(frs);
        std::this_thread::sleep_for(std::chrono::milliseconds(dt));
    }
}

zlc::Empty FrankaArmProxy::setControlMode(const std::string& mode)
{
    if (current_control_mode_ != nullptr)
    {
        zlc::info("Stopping previous control mode...");
        current_control_mode_->stopControl();
    }
    zlc::info("Switching to control mode: {}", mode);
    current_control_mode_ = control_modes_.at(mode).get();
    current_control_mode_->startControl();
    return zlc::empty;
}

FrankaArmState FrankaArmProxy::getFrankaArmState(const zlc::Empty&)
{
    return FrankaArmState(current_state.read());
}

std::string FrankaArmProxy::getFrankaArmControlMode(const zlc::Empty&)
{
    if (!current_control_mode_)
    {
        throw std::runtime_error("No active control mode");
    }
    return current_control_mode_->getModeName();
}
