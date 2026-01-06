// #pragma once
// #include <zmq.hpp>
// #include <thread>
// #include <atomic>
// #include <mutex>
// #include <string>
// #include <memory>


// #include <franka/gripper.h>
// #include <franka/robot_state.h>
// #include "control_mode/abstract_control_mode.hpp"
// #include "utils/atomic_double_buffer.hpp"
// #include "utils/franka_config.hpp"

// #include "protocol/grasp_command.hpp"
// #include <iostream>

// #include <zerolancom/zerolancom.hpp>
// #include <zerolancom/sockets/publisher.hpp>
// #include <msgpack.hpp>

// enum class FrankaGripperFlag {
//     STOP = 0,
//     STOPPING = 1,
//     CLOSING = 2,
//     OPENING = 3,
// };


// class FrankaGripperProxy {

// public:
//     // Constructor & Destructor
//     explicit FrankaGripperProxy(const FrankaGripperConfigData& config):
//         state_pub_socket_(ZmqContext::instance(), ZMQ_PUB),//gripper state publish socket
//         is_running(false),
//         is_on_control_mode(false),
//         gripper_flag(FrankaGripperFlag::STOP),
//         current_state_(AtomicDoubleBuffer<franka::GripperState>(franka::GripperState{})),
//         command_(AtomicDoubleBuffer<protocol::GraspCommand>(protocol::GraspCommand{})),
//         config_(config)
//     {
//         gripper_ip_ = config_.gripper_ip;
//         state_pub_addr_ = config_.gripper_state_pub_addr;
//         // Bind state pub socket
//         zlc::info("Gripper state publisher bound to {}", state_pub_addr_);
//         state_pub_socket_.bind(state_pub_addr_);
//         // Removed service_registry_.bindSocket as ZeroLanCom handles this
//         //initialize franka gripper
// # if !LOCAL_TESTING
//         gripper_ = std::make_shared<franka::Gripper>(gripper_ip_);
//         gripper_->homing();
//         current_state_.write(gripper_->readOnce());
//         command_.write(protocol::GraspCommand{
//             current_state_.read().width,
//             static_cast<float>(config_.gripper_default_speed_slow),
//             static_cast<float>(config_.gripper_default_force)
//         });
// # else
//         current_state_.write(franka::GripperState{0.0f, 0.0f, false, false, franka::Duration{}});
//         command_.write(protocol::GraspCommand{
//             0.0f,
//             static_cast<float>(config_.gripper_default_speed_slow),
//             static_cast<float>(config_.gripper_default_force)
//         });
// # endif
//         initializeService();
//     };
//     ~FrankaGripperProxy() {
//         stop();
//     };

//     // Core server operations
//     void start() {
//         is_running = true;
//         zlc::info("Gripper proxy running flag set to {}", is_running.load());
//         state_pub_thread_ = std::thread(&FrankaGripperProxy::statePubThread, this);
//         command_.write(protocol::GraspCommand{
//             (float)current_state_.read().width,
//             static_cast<float>(config_.gripper_default_speed_fast),
//             static_cast<float>(config_.gripper_default_force)
//         });
//         state_pub_thread_ = std::thread(&FrankaGripperProxy::statePubThread, this);
//         check_thread_ = std::thread(&FrankaGripperProxy::checkWidthThread, this);
//         control_thread_ = std::thread(&FrankaGripperProxy::controlLoopThread, this);
//     };

//     void stop() {
//         zlc::info("Stopping FrankaGripperProxy...");
//         is_running = false;
//         is_on_control_mode = false;
//         if (state_pub_thread_.joinable()) state_pub_thread_.join();
//         if (control_thread_.joinable()) control_thread_.join();
//         if (check_thread_.joinable()) check_thread_.join();
//         if (command_sub_thread_.joinable()) command_sub_thread_.join();

//         // Removed service_registry_.stop() as ZeroLanCom handles this
//         gripper_.reset();
//         zlc::info("FrankaGripperProxy stopped successfully.");
//     };

// private:
//     // Initialization
//     void initializeService() {
//         // Register service handlers
//         node_.registerServiceHandler("START_FRANKA_GRIPPER_CONTROL", &FrankaGripperProxy::startFrankaGripperControl, this);
//         node_.registerServiceHandler("GET_FRANKA_GRIPPER_STATE_PUB_PORT", &FrankaGripperProxy::getFrankaGripperStatePubPort, this);
//     };

//     std::string gripper_ip_;
//     std::string service_addr_;
//     std::string state_pub_addr_;
//     // Franka robot
//     std::shared_ptr<franka::Gripper> gripper_;
//     // ZMQ communication
//     zmq::socket_t state_pub_socket_;//arm state publish socket

//     // Threading
//     std::thread state_pub_thread_;
//     std::thread control_thread_;
//     std::thread check_thread_;
//     std::thread command_sub_thread_;
    
//     // Threading Tasks
//     void controlLoopThread() {
//         while (is_running) {
//             float current_width = current_state_.read().width;
//             float target_width = command_.read().width;
//             if (gripper_flag.load() == FrankaGripperFlag::STOPPING) {
//                 std::this_thread::sleep_for(std::chrono::milliseconds(10));
//                 continue;
//             }
//             try
//             {
//                 if (current_width > target_width + config_.gripper_default_close_open_threshold) {
//                     zlc::info("[Gripper Close] Closing gripper to target width: {}", target_width);
//                     gripper_flag.store(FrankaGripperFlag::CLOSING);
//                     gripper_->grasp(target_width, command_.read().speed, 60.0);
//                 } else if (current_width < target_width - config_.gripper_default_close_open_threshold) {
//                     zlc::info("[Gripper Open] Opening gripper to target width: {}", target_width);
//                     gripper_flag.store(FrankaGripperFlag::OPENING);
//                     gripper_->move(target_width, command_.read().speed);
//                 }
//             }
//             catch(const std::exception& e)
//             {
//                 zlc::error("{}", e.what());
//             }
//             gripper_flag.store(FrankaGripperFlag::STOP);
//         }
//     };

//     void statePubThread() {
//         zerolancom::Publisher<> state_pub(ZmqContext::instance());;
//         while (is_running) {
// #if !LOCAL_TESTING
//             franka::GripperState gs = gripper_->readOnce();
//             current_state_.write(gs);

//             // Convert to FrankaGripperState and serialize with msgpack
//             FrankaGripperState gripper_state_msg;
//             gripper_state_msg.width = gs.width;
//             gripper_state_msg.max_width = gs.max_width;
//             gripper_state_msg.is_grasped = gs.is_grasped;
//             gripper_state_msg.temperature = gs.temperature;

//             msgpack::sbuffer buffer;
//             msgpack::pack(buffer, gripper_state_msg);

//             state_pub_socket_.send(zmq::buffer(buffer.data(), buffer.size()), zmq::send_flags::none);
// #endif
//             const int rate = config_.gripper_pub_rate_hz > 0 ? config_.gripper_pub_rate_hz : GRIPPER_PUB_RATE_HZ;
//             std::this_thread::sleep_for(std::chrono::milliseconds(1000 / rate));
//         }
//         try {
//             state_pub_socket_.close();
//         } catch (const zmq::error_t& e) {
//             zlc::error("[ZMQ ERROR] {}", e.what());
//         }
//     };

//     void checkWidthThread() {
//         while (is_running) {
// #if !LOCAL_TESTING
//             float current_width = current_state_.read().width;
//             float target_width = command_.read().width;
//             if (
//                 gripper_flag.load() == FrankaGripperFlag::STOPPING
//             || (gripper_flag.load() == FrankaGripperFlag::CLOSING && current_width >= target_width + config_.gripper_default_close_open_threshold)
//             || (gripper_flag.load() == FrankaGripperFlag::OPENING && current_width <= target_width - config_.gripper_default_close_open_threshold)
//             || (gripper_flag.load() == FrankaGripperFlag::STOP && std::abs(current_width - target_width) <= config_.gripper_default_close_open_threshold)
//             ) {
//                 std::this_thread::sleep_for(std::chrono::milliseconds(10));
//                 continue;
//             }
//             try
//             {
//                 zlc::info("[Gripper Stop] Stopping gripper at current width: {}", current_width);
//                 gripper_flag.store(FrankaGripperFlag::STOP);
//                 gripper_->stop();
//             }
//             catch(const std::exception& e)
//             {
//                 zlc::error("{}", e.what());
//             }
// #endif
//             std::this_thread::sleep_for(std::chrono::milliseconds(1));
//         }
//     };

//     void commandSubThread(const std::string& command_sub_addr) {
//         is_on_control_mode = true;
//         zmq::socket_t sub_socket_(ZmqContext::instance(), ZMQ_SUB);
//         sub_socket_.set(zmq::sockopt::rcvtimeo, config_.gripper_command_rcvtimeo_ms); // timeout
//         sub_socket_.set(zmq::sockopt::subscribe, ""); // Subscribe to all messages
//         sub_socket_.connect(command_sub_addr);
//         while (is_running && is_on_control_mode) {
//             sub_socket_.set(zmq::sockopt::subscribe, ""); // Subscribe to all messages
//             zmq::message_t message;
//             if (!sub_socket_.recv(message, zmq::recv_flags::none)) {
//                 continue;
//             }
//             protocol::ByteView data{
//                 static_cast<const uint8_t*>(message.data()),
//                 message.size()
//             };
//             command_.write(protocol::decode<protocol::GraspCommand>(data));
//             std::this_thread::sleep_for(std::chrono::milliseconds(10));
//         }
//     };


//     // Synchronization
//     std::atomic<bool> is_running; // for threads
//     std::atomic<bool> is_moving;
//     std::atomic<bool> is_on_control_mode;
//     std::atomic<FrankaGripperFlag> gripper_flag;
    
//     AtomicDoubleBuffer<franka::GripperState> current_state_;
//     AtomicDoubleBuffer<protocol::GraspCommand> command_;
//     FrankaGripperConfigData config_;
//     zerolancom::ZeroLanComNode& node_;
//     void startFrankaGripperControl(const std::string& command_sub_addr) {
//         if (is_on_control_mode) {
//             zlc::warn("[FrankaGripperProxy] Already in control mode, stopping previous command subscriber.");
//             is_on_control_mode = false;
//             if (command_sub_thread_.joinable()) command_sub_thread_.join();
//         }
//         command_sub_thread_ = std::thread(&FrankaGripperProxy::commandSubThread, this, command_sub_addr);
//     };
//     const std::string& getFrankaGripperStatePubPort() {
//         return state_pub_addr_;
//     };

//     // TODO: put all the Constants to a config file
//     static constexpr int STATE_PUB_RATE_HZ = 100;
//     static constexpr int GRIPPER_PUB_RATE_HZ = 100;
//     static constexpr int SOCKET_TIMEOUT_MS = 100;
//     static constexpr int MAX_MESSAGE_SIZE = 4096;
// };
