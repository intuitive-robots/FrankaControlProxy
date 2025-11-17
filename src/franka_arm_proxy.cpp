#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <algorithm>
#include "franka_arm_proxy.hpp"
#include "protocol/codec.hpp"
#include "protocol/msg_id.hpp"
#include "protocol/msg_header.hpp"
#include "utils/franka_config.hpp"
#include "debugger/state_debug.hpp"
#include "protocol/codec.hpp"
#include "protocol/mode_id.hpp"
#include "protocol/request_result.hpp"
#include "control_mode/control_mode.hpp"
#include "utils/service_registry.hpp"



static std::atomic<bool> running_flag{true};  // let ctrl-c stop the server
static void signalHandler(int signum) {
    std::cout << "\n[INFO] Caught signal " << signum << ", shutting down..." << std::endl;
    running_flag = false;
}
// Todo: may add to config file later
franka::RobotState default_state = []{
    franka::RobotState state;
    state.q = {{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}};
    state.O_T_EE = {{
        1.0, 0.0, 0.0, 0.3,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.5,
        0.0, 0.0, 0.0, 1.0
    }};
    return state;
}();

FrankaArmProxy::FrankaArmProxy(const std::string& config_path)
    : state_pub_socket_(ZmqContext::instance(), ZMQ_PUB),//arm state publish socket
      is_running(false),
      current_state_(AtomicDoubleBuffer<franka::RobotState>(default_state)),
      service_registry_()
    {
    FrankaConfig config(config_path);
    // type_ = config.getValue("type", "Arm"); // Default to "Arm" if not specified
    robot_ip_ = config.getValue("robot_ip");
    //bind state pub socket
    state_pub_addr_ = config.getValue("state_pub_addr");
    std::cout<<"state_pub: "<< state_pub_addr_ <<std::endl;
    state_pub_socket_.bind(state_pub_addr_);
    service_registry_.bindSocket(config.getValue("service_addr"));
    //initialize franka robot
#if !LOCAL_TESTING
    try
    {
        robot_ = std::make_shared<franka::Robot>(robot_ip_);
        model_ = std::make_shared<franka::Model>(robot_->loadModel());
    }
    catch(const franka::NetworkException& e)
    {
        std::cerr << e.what() << '\n';
        this->stop();
    }
#endif
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
    // Register service handlers
    service_registry_.registerHandler("SET_FRANKA_ARM_CONTROL_MODE", this, &FrankaArmProxy::setControlMode);
    service_registry_.registerHandler("GET_FRANKA_ARM_STATE", this, &FrankaArmProxy::getFrankaArmState);
    service_registry_.registerHandler("GET_FRANKA_ARM_CONTROL_MODE", this, &FrankaArmProxy::getFrankaArmControlMode);
    service_registry_.registerHandler("GET_FRANKA_ARM_STATE_PUB_PORT", this, &FrankaArmProxy::getFrankaArmStatePubPort);
    // service_registry_.registerHandler("MOVE_FRANKA_ARM_TO_JOINT_POSITION", this, &FrankaArmProxy::moveFrankaArmToJointPosition);
    // service_registry_.registerHandler("MOVE_FRANKA_ARM_TO_CARTESIAN_POSITION", this, &FrankaArmProxy::moveFrankaArmToCartesianPosition);
    service_registry_.start();
}


FrankaArmProxy::~FrankaArmProxy() {
    stop();
}


bool FrankaArmProxy::start(){
    is_running = true;
    std::cout << is_running <<"control"<< std::endl;
    std::cout << robot_<<"robot"<< std::endl;
#if LOCAL_TESTING
    current_state_.write(default_state);
#else
    // current_state_.write(robot_->readOnce());
#endif
    state_pub_thread_ = std::thread(&FrankaArmProxy::statePublishThread, this);
    // std::cout << "done arm state pub"<< std::endl;
    // service_thread_ = std::thread(&FrankaArmProxy::responseSocketThread, this);
    // std::cout << "done service"<< std::endl;
    return true;
}

void FrankaArmProxy::stop() {
    std::cout << "[INFO] Stopping FrankaArmProxy..." << std::endl;
    is_running = false;
    if (current_mode_)
        current_mode_->stop();
    if (state_pub_thread_.joinable()) state_pub_thread_.join();
    // if (service_thread_.joinable()) service_thread_.join();
    // try close ZeroMQ sockets
    try {
        state_pub_socket_.close();
    } catch (const zmq::error_t& e) {
        std::cerr << "[ZMQ ERROR] " << e.what() << "\n";
    }
    // wait for closing
    service_registry_.stop();
#if !LOCAL_TESTING
    robot_.reset();
    model_.reset();
#endif
    current_mode_ = nullptr;// reset current control mode
    std::cout << "[INFO] FrankaArmProxy stopped successfully." << std::endl;
}

// Main loop for processing requests, ctrl-c to stop the server
void FrankaArmProxy::spin() {
    std::signal(SIGINT, signalHandler);  //  Catch Ctrl+C to stop the server
    std::cout << "[INFO] Entering main spin loop. Press Ctrl+C to exit." << std::endl;
    std::cout << "running_flag " << running_flag << std::endl;
    while (running_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    stop(); 
    std::cout << "[INFO] Shutdown complete.\n";
}

// publish threads
void FrankaArmProxy::statePublishThread() {
    while (is_running) {
        const franka::RobotState rs = current_state_.read();

        // Encode payload & wrap with header
        const std::vector<uint8_t> payload = protocol::encode(rs);
        // Publish over ZMQ PUB socket
        state_pub_socket_.send(zmq::buffer(payload), zmq::send_flags::none);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / STATE_PUB_RATE_HZ));
        // std::cout << "[FrankaArmProxy] Published state message, size = " << frame.size() << " bytes." << std::endl;//debug
    }
}

void FrankaArmProxy::setControlMode(const protocol::FrankaArmControlMode& mode) {
    if (current_mode_.get() != nullptr) {
        std::cout << "[Info] Stopping previous control mode...\n";
        current_mode_->stop();  // stopMotion + is_running_ = false
    }
    std::cout << "[Info] Switching to control mode: " << protocol::toString(mode.id) << " with URL: " << mode.url << std::endl;
    current_mode_ = ControlModeFactory::create(mode.id);
#if !LOCAL_TESTING
    current_mode_->setRobot(robot_);
    current_mode_->setModel(model_);
#endif
    current_mode_->setCurrentStateBuffer(&current_state_);
    current_mode_->setupCommandSubscription(mode.url);
    current_mode_->start();
}

franka::RobotState FrankaArmProxy::getFrankaArmState() {
    return current_state_.read();
}

uint8_t FrankaArmProxy::getFrankaArmControlMode() {
    if (!current_mode_) {
        throw std::runtime_error("No active control mode");
    }
    return static_cast<uint8_t>(current_mode_->getModeID());
}
const std::string& FrankaArmProxy::getFrankaArmStatePubPort() {
    // return state_pub_addr_
    return state_pub_addr_;
}
