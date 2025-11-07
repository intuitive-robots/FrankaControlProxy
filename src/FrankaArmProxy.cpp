#include "FrankaArmProxy.hpp"
#include "protocol/codec.hpp"
#include "protocol/franka_arm_state.hpp"
#include "protocol/franka_gripper_state.hpp"
#include "protocol/msg_id.hpp"
#include "protocol/message_header.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <algorithm>
#include "RobotConfig.hpp"
#include "debugger/state_debug.hpp"
#include "protocol/mode_id.hpp"
#include "protocol/request_result.hpp"
#include "control_mode/control_mode_factory.hpp"
#include "utils/ServiceRegistry.hpp"


static std::atomic<bool> running_flag{true};  // let ctrl-c stop the server
static void signalHandler(int signum) {
    std::cout << "\n[INFO] Caught signal " << signum << ", shutting down..." << std::endl;
    running_flag = false;
}
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
    : context_(1), // Initialize ZMQ context with 2 I/O threads,it can be adjusted
    state_pub_socket_(context_, ZMQ_PUB),
    res_socket_(context_, ZMQ_REP), 
    is_running(false) 
    {
        initialize(config_path);
    }

void FrankaArmProxy::initialize(const std::string& filename) {
    RobotConfig config(filename);
    type_ = config.getValue("type", "Arm"); // Default to "Arm" if not specified
    robot_ip_ = config.getValue("robot_ip");
    res_socket_.set(zmq::sockopt::rcvtimeo, SOCKET_TIMEOUT_MS);
    state_pub_addr_ = config.getValue("state_pub_addr");
    std::cout<<"state_pub"<<state_pub_addr_ <<std::endl;
    state_pub_socket_.bind(state_pub_addr_);
    service_addr_ = config.getValue("service_addr");
    //std::cout<<"service_addr"<<service_addr_ <<std::endl;
    res_socket_.bind(service_addr_);
    robot_ = std::make_shared<franka::Robot>(robot_ip_);
    model_ = std::make_shared<franka::Model>(robot_->loadModel());
 
    // Register service handlers
    service_registry_ = ServiceRegistry<FrankaArmProxy, protocol::MessageHeader>();
    service_registry_.registerHandler<const std::string&, protocol::RequestResult>(&FrankaArmProxy::setControlMode);

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
    std::cout << "done arm pub"<< std::endl;
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
    while (is_running) {
        protocol::FrankaArmState proto_state = protocol::FrankaArmState::fromRobotState(getCurrentState());
        auto msg = protocol::encodeStateMessage(proto_state);
        state_pub_socket_.send(zmq::buffer(msg), zmq::send_flags::none);
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
        std::cout << "[FrankaArmProxy] Received request: msg size = " << req_data.size() << std::endl;
        //std::string response;
        std::vector<uint8_t> response;
        handleServiceRequest(req_data, response);
        //send response
        res_socket_.send(zmq::buffer(response), zmq::send_flags::none);
        std::cout << "[FrankaArmProxy] Sent response: msg size = " << response.size() << std::endl;
    }
}

//TODO: error message have not done
void FrankaArmProxy::handleServiceRequest(const std::vector<uint8_t>& request, std::vector<uint8_t>& response) {
    //check the request size 
    using namespace protocol;
    if (request.size() < MessageHeader::SIZE) {
        response = encodeErrorMessage(0x01);
        std::cerr << "[FrankaArmProxy] Invalid request size: " << request.size() << ". Expected at least 12 bytes." << std::endl;
        return;
    }

    //parse the header
    const uint8_t* data = reinterpret_cast<const uint8_t*>(request.data());
    MessageHeader header = MessageHeader::decode(data);
    // if (request.size() != 4 + header.len) {
    //     response = encodeErrorMessage(0x02);
    //     return;
    // }
    
    const std::vector<uint8_t> payload = data + MessageHeader::SIZE;
    std::vector<uint8_t> resp;
    uint8_t command = payload[0];
    //deal with different kinds of msg
    service_registry_.handleMessage(header, payload, response);
    std::cout << "[FrankaArmProxy] Response prepared, size = " << response.size() << std::endl;
    //response.assign(reinterpret_cast<const char*>(resp.data()), resp.size());
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

