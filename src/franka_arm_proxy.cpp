#include "franka_arm_proxy.hpp"
#include "protocol/codec.hpp"
#include "protocol/msg_id.hpp"
#include "protocol/msg_header.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <algorithm>
#include "robot_config.hpp"
#include "debugger/state_debug.hpp"
#include "protocol/mode_id.hpp"
#include "protocol/request_result.hpp"
#include "control_mode/control_mode_factory.hpp"
#include "utils/service_registry.hpp"


static std::atomic<bool> running_flag{true};  // let ctrl-c stop the server
static void signalHandler(int signum) {
    std::cout << "\n[INFO] Caught signal " << signum << ", shutting down..." << std::endl;
    running_flag = false;
}
// Todo: may add to config file later
franka::RobotState default_state = []{
    franka::RobotState state;
    state.q = {{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}};  //default joint positions
    state.O_T_EE = {{
        1.0, 0.0, 0.0, 0.3,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.5,
        0.0, 0.0, 0.0, 1.0
    }};
    return state;
}();


////main Proxy
FrankaArmProxy::FrankaArmProxy(const std::string& config_path)
    : context_(1), 
    state_pub_socket_(context_, ZMQ_PUB),//arm state publish socket
    res_socket_(context_, ZMQ_REP), //service socket
    is_running(false) 
    {
        initialize(config_path);
    }

void FrankaArmProxy::initialize(const std::string& filename) {
    RobotConfig config(filename);
    // type_ = config.getValue("type", "Arm"); // Default to "Arm" if not specified
    robot_ip_ = config.getValue("robot_ip");
    //bind state pub socket
    state_pub_addr_ = config.getValue("state_pub_addr");
    std::cout<<"state_pub"<<state_pub_addr_ <<std::endl;
    state_pub_socket_.bind(state_pub_addr_);
    //bind service socket
    res_socket_.set(zmq::sockopt::rcvtimeo, SOCKET_TIMEOUT_MS);
    service_addr_ = config.getValue("service_addr");
    //std::cout<<"service_addr"<<service_addr_ <<std::endl;//debug
    res_socket_.bind(service_addr_);
    //initialize franka robot
    robot_ = std::make_shared<franka::Robot>(robot_ip_);
    model_ = std::make_shared<franka::Model>(robot_->loadModel());
 
    // Register service handlers
    service_registry_ = ServiceRegistry();
    service_registry_.registerHandler("setControlMode", this, &FrankaArmProxy::setControlMode);

    service_registry_.registerHandler(protocol::MsgID::GET_FRANKA_ARM_STATE, this, &FrankaArmProxy::GET_FRANKA_ARM_STATE);
    service_registry_.registerHandler(protocol::MsgID::GET_FRANKA_ARM_CONTROL_MODE, this, &FrankaArmProxy::GET_FRANKA_ARM_CONTROL_MODE);
    service_registry_.registerHandler(protocol::MsgID::GET_FRANKA_ARM_STATE_PUB_PORT, this, &FrankaArmProxy::GET_FRANKA_ARM_STATE_PUB_PORT);
    service_registry_.registerHandler(protocol::MsgID::MOVE_FRANKA_ARM_TO_JOINT_POSITION, this, &FrankaArmProxy::MOVE_FRANKA_ARM_TO_JOINT_POSITION);
    service_registry_.registerHandler(protocol::MsgID::MOVE_FRANKA_ARM_TO_CARTESIAN_POSITION, this, &FrankaArmProxy::MOVE_FRANKA_ARM_TO_CARTESIAN_POSITION);

}

FrankaArmProxy::~FrankaArmProxy() {
    stop();
}


bool FrankaArmProxy::start(){
    is_running = true;
    std::cout << is_running <<"control"<< std::endl;
    std::cout << robot_<<"robot"<< std::endl;
    current_state_.write(robot_->readOnce());
    state_pub_thread_ = std::thread(&FrankaArmProxy::statePublishThread, this);
    std::cout << "done arm state pub"<< std::endl;
    service_thread_ = std::thread(&FrankaArmProxy::responseSocketThread, this);
    std::cout << "done service"<< std::endl;
    return true;
}

void FrankaArmProxy::stop() {
    is_running = false;
    current_mode_->stop();
    // try close ZeroMQ sockets
    try {
        state_pub_socket_.close();
        res_socket_.close();
    } catch (const zmq::error_t& e) {
        std::cerr << "[ZMQ ERROR] " << e.what() << "\n";
    }
    // wait for closing
    if (state_pub_thread_.joinable()) state_pub_thread_.join();
    if (service_thread_.joinable()) service_thread_.join();
    robot_.reset();
    model_.reset();
    current_mode_ = nullptr;// reset current control mode
}

// Main loop for processing requests, ctrl-c to stop the server
void FrankaArmProxy::spin() {
    std::signal(SIGINT, signalHandler);  //  Catch Ctrl+C to stop the server
    std::cout << "[INFO] Entering main spin loop. Press Ctrl+C to exit." << std::endl;
    std::cout << "running_flag" <<running_flag << std::endl;
    while (running_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    stop(); 
    std::cout << "[INFO] Shutdown complete.\n";
}

// with RobotConfig
void FrankaArmProxy::displayConfig() const {
    std::cout << proxy_config_ << std::endl;
}

// publish threads
void FrankaArmProxy::statePublishThread() {
    using namespace protocol;
    while (is_running) {
        const franka::RobotState rs = current_state_.read();

        // Encode payload & wrap with header
        const std::vector<uint8_t> payload = encode(rs);
        const MsgHeader header = createHeader(static_cast<uint8_t>(MsgID::FRANKA_ARM_STATE_PUB),
                                              static_cast<uint16_t>(payload.size()));
        std::vector<uint8_t> frame = encodeMessage(header, payload);
        // Publish over ZMQ PUB socket
        state_pub_socket_.send(zmq::buffer(frame), zmq::send_flags::none);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / STATE_PUB_RATE_HZ));
    }
}

//response socket thread, used for service request
void FrankaArmProxy::responseSocketThread() {
    while (is_running) {
        //get request from client
        zmq::message_t request;
        if (!res_socket_.recv(request, zmq::recv_flags::none)) continue;//skip,if fail
        std::vector<uint8_t> req_data(static_cast<uint8_t*>(request.data()),//begin
                                      static_cast<uint8_t*>(request.data()) + request.size());//end
        std::cout << "[FrankaArmProxy] Received request: msg size = " << req_data.size() << std::endl;//debug
        //std::string response;
        std::vector<uint8_t> response;
        handleServiceRequest(req_data, response);
        //send response
        res_socket_.send(zmq::buffer(response), zmq::send_flags::none);
        std::cout << "[FrankaArmProxy] Sent response: msg size = " << response.size() << std::endl;//debug
    }
}

void FrankaArmProxy::handleServiceRequest(const std::vector<uint8_t>& request, std::vector<uint8_t>& response) {
    using namespace protocol;
        // Validate header length
        if (request.size() < MsgHeader::SIZE) {
            response = RequestResult(RequestResultCode::INVALID_ARG, "Bad header").encodeMessage();
            return;
        }
        const uint8_t* data = reinterpret_cast<const uint8_t*>(request.data());
        const MsgHeader req_header = MsgHeader::decode(data);//get header

        // Validate payload length
        const size_t expect = static_cast<size_t>(MsgHeader::SIZE) + req_header.payload_length;
        if (request.size() != expect) {
            response = RequestResult(RequestResultCode::INVALID_ARG, "Truncated payload").encodeMessage();
            return;
        }
        std::vector<uint8_t> payload(data + MsgHeader::SIZE, data + MsgHeader::SIZE + req_header.payload_length);//get payload

        // get handler response payload
        std::vector<uint8_t> resp_payload;
        try {
            resp_payload = service_registry_.handleMessage(req_header, payload);
        } catch (const std::exception& e) {
            response = RequestResult(RequestResultCode::FAIL, e.what()).encodeMessage();
            return;
        }
        // respond with proper header
        const MsgHeader resp_header = createHeader(req_header.message_type,
                                                static_cast<uint16_t>(resp_payload.size()));
        response = encodeMessage(resp_header, resp_payload);
        }

protocol::RequestResult FrankaArmProxy::setControlMode(const std::string& mode) {
    if (current_mode_.get() != nullptr) {
        std::cout << "[Info] Stopping previous control mode...\n";
        current_mode_->stop();  // stopMotion + is_running_ = false
    }
    current_mode_ = ControlModeFactory::create(mode);
    current_mode_->setRobot(robot_);
    current_mode_->setModel(model_);
    current_mode_->start();
    std::cout << "[Info] Switched to control mode: " << mode << std::endl;
    return protocol::RequestResult(protocol::RequestResultCode::SUCCESS);
}

franka::RobotState FrankaArmProxy::GET_FRANKA_ARM_STATE() {
    return current_state_.read();
}

uint8_t FrankaArmProxy::GET_FRANKA_ARM_CONTROL_MODE() {
    if (!current_mode_) {
        throw std::runtime_error("No active control mode");
    }
    return static_cast<uint8_t>(current_mode_->getModeID());
}
uint16_t FrankaArmProxy::GET_FRANKA_ARM_STATE_PUB_PORT() {
    // Extract port from state_pub_addr_
    std::string prefix = "tcp://*:";
    if (state_pub_addr_.find(prefix) != 0) {
        throw std::runtime_error("Invalid state_pub_addr_ format");
    }
    std::string port_str = state_pub_addr_.substr(prefix.size());
    return static_cast<uint16_t>(std::stoi(port_str));
}
