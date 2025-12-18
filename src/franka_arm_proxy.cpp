#include <array>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <algorithm>
#include <thread>
#include "utils/logger.hpp"
#include "franka_arm_proxy.hpp"
#include "protocol/msg.hpp"
#include "utils/franka_config.hpp"
#include "debugger/state_debug.hpp"
#include "protocol/codec.hpp"
#include "protocol/mode_id.hpp"
#include "protocol/request_result.hpp"
#include "control_mode/control_mode.hpp"

#include <zerolancom/zerolancom.hpp>

static std::atomic<bool> running_flag{true};  // let ctrl-c stop the server
static void signalHandler(int signum) {
    LOG_INFO("Caught signal {}, shutting down...", signum);
    running_flag = false;
}
namespace {
franka::RobotState makeDefaultState(const FrankaConfigData& cfg) {
    franka::RobotState state{};
    std::array<double, 7> q{};
    const auto& q_src = cfg.arm_default_state_q.size() == 7
                        ? cfg.arm_default_state_q
                        : std::vector<double>{{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}};
    std::copy(q_src.begin(), q_src.begin() + 7, q.begin());
    state.q = q;

    std::array<double, 16> pose = cfg.arm_default_state_O_T_EE;
    if (pose == std::array<double, 16>{}) {
        pose = {{
            1.0, 0.0, 0.0, 0.3,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.5,
            0.0, 0.0, 0.0, 1.0
        }};
    }
    state.O_T_EE = pose;
    return state;
}
}

FrankaArmProxy::FrankaArmProxy(const FrankaConfigData& config)
    : is_running(false),
      current_state(AtomicDoubleBuffer<franka::RobotState>(makeDefaultState(config))),
      config_(config),
      default_state_(makeDefaultState(config))
    {
    robot_ip_ = config_.robot_ip;
    //initialize franka robot
    try
    {
        robot_ = std::make_shared<franka::Robot>(robot_ip_);
        model_ = std::make_shared<franka::Model>(robot_->loadModel());
    }
    catch(const franka::NetworkException& e)
    {
        LOG_ERROR("{}", e.what());
        this->stop();
    }
    //initialize control modes
    initializeControlMode();
    // Register service handlers
    initializeService();
    setControlMode(protocol::FrankaArmControlMode{protocol::ModeID::IDLE, ""});
}

void FrankaArmProxy::initializeControlMode() {
    ControlModeFactory::registerMode(protocol::toString(protocol::ModeID::IDLE), []() { return std::make_shared<IdleControlMode>(); });
    ControlModeFactory::registerMode(protocol::toString(protocol::ModeID::JOINT_POSITION), []() { return std::make_shared<JointPositionMode>(); });
    ControlModeFactory::registerMode(protocol::toString(protocol::ModeID::JOINT_VELOCITY), []() { return std::make_shared<JointVelocityMode>(); });
    ControlModeFactory::registerMode(protocol::toString(protocol::ModeID::CARTESIAN_POSE), []() { return std::make_shared<CartesianPoseMode>(); });
    ControlModeFactory::registerMode(protocol::toString(protocol::ModeID::CARTESIAN_VELOCITY), []() { return std::make_shared<CartesianVelocityMode>(); });
    ControlModeFactory::registerMode(protocol::toString(protocol::ModeID::JOINT_TORQUE), []() { return std::make_shared<HumanControlMode>(); });
}


void FrankaArmProxy::initializeService() {
    zlc::registerServiceHandler("SET_FRANKA_ARM_CONTROL_MODE", &FrankaArmProxy::setControlMode, this);
    zlc::registerServiceHandler("GET_FRANKA_ARM_STATE", &FrankaArmProxy::getFrankaArmState, this);
    zlc::registerServiceHandler("GET_FRANKA_ARM_CONTROL_MODE", &FrankaArmProxy::getFrankaArmControlMode, this);
    // zlc::registerServiceHandler("MOVE_FRANKA_ARM_TO_JOINT_POSITION", &FrankaArmProxy::moveFrankaArmToJointPosition, this);
    // zlc::registerServiceHandler("MOVE_FRANKA_ARM_TO_CARTESIAN_POSITION", &FrankaArmProxy::moveFrankaArmToCartesianPosition, this);
}


FrankaArmProxy::~FrankaArmProxy() {
    stop();
}


bool FrankaArmProxy::start(){
    is_running = true;
    LOG_INFO("Arm proxy running flag set to {}", is_running.load());
    LOG_INFO("Robot interface initialized: {}", robot_ != nullptr);
    current_state.write(robot_->readOnce());
    state_pub_thread = std::thread(&FrankaArmProxy::statePublishThread, this);
    LOG_INFO("FrankaArmProxy started successfully.");
    return true;
}

void FrankaArmProxy::stop() {
    LOG_INFO("Stopping FrankaArmProxy...");
    is_running = false;
    // TODO: remove all the services and subscribers
    if (current_mode_)
        current_mode_->stop();
    if (state_pub_thread.joinable()) state_pub_thread.join();
    // wait for closing
    robot_.reset();
    model_.reset();
    current_mode_ = nullptr;// reset current control mode
    LOG_INFO("FrankaArmProxy stopped successfully.");
}

// Main loop for processing requests, ctrl-c to stop the server
void FrankaArmProxy::spin() {
    std::signal(SIGINT, signalHandler);  //  Catch Ctrl+C to stop the server
    LOG_INFO("Entering main spin loop. Press Ctrl+C to exit.");
    LOG_INFO("Current running flag: {}", running_flag.load());
    while (running_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    stop(); 
    LOG_INFO("Shutdown complete.");
}

// publish threads
void FrankaArmProxy::statePublishThread() {
    zlc::Publisher<FrankaRobotState> state_pub("FRANKA_ARM_STATE_PUB");
    while (is_running) {
        const franka::RobotState rs = current_state.read();
        FrankaRobotState frs(rs);
        state_pub.publish(frs);
        // TODO: use config time and make sure the rate is correct
        const int rate = config_.arm_state_pub_rate_hz > 0 ? config_.arm_state_pub_rate_hz : STATE_PUB_RATE_HZ;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / rate));
    }
}

void FrankaArmProxy::setControlMode(const protocol::FrankaArmControlMode& mode) {
    if (current_mode_.get() != nullptr) {
        LOG_INFO("Stopping previous control mode...");
        current_mode_->stop();  // stopMotion + is_running_ = false
    }
    LOG_INFO("Switching to control mode: {} (command URL: {})", protocol::toString(mode.id), mode.url);
    current_mode_ = ControlModeFactory::create(mode.id);
    current_mode_->init(robot_, model_);
    current_mode_->setCurrentStateBuffer(current_state);
    current_mode_->setCommandSubscription();
    current_mode_->start();
}

franka::RobotState FrankaArmProxy::getFrankaArmState() {
    return current_state.read();
}

uint8_t FrankaArmProxy::getFrankaArmControlMode() {
    if (!current_mode_) {
        throw std::runtime_error("No active control mode");
    }
    return static_cast<uint8_t>(current_mode_->getModeID());
}

