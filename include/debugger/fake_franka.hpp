// #include <franka/robot.h>

// #include <atomic>
// #include <thread>
// #include <zerolancom/zerolancom.hpp>

// class FakeFrankaModel
// {
//   public:
//     FakeFrankaModel()
//     {
//         zlc::info("Initialized FakeFrankaModel for testing purposes.");
//     }
//     ~FakeFrankaModel() = default;
//     // Add other necessary fake methods as needed for testing
//     std::array<double, 7> coriolis(const franka::RobotState& state)
//     {
//         return std::array<double, 7>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//     }
// };

// class FakeFrankaRobot
// {
//   public:
//     FakeFrankaRobot() = delete;
//     FakeFrankaRobot(const std::string&)
//     {
//         zlc::info("Initialized FakeFrankaRobot for testing purposes.");
//     }

//     ~FakeFrankaRobot()
//     {
//         zlc::info("Destroyed FakeFrankaRobot.");
//     }

//     void automaticErrorRecovery()
//     {
//         zlc::info("FakeFrankaRobot: automaticErrorRecovery called.");
//     }

//     template <typename ControllerCallback>
//     void control(ControllerCallback callback, bool limit_rate = true,
//                  double cutoff_frequency = (100.0))
//     {
//         zlc::info("FakeFrankaRobot: control loop started.");
//         isOnControlLoop = true;
//         while (isOnControlLoop)
//         {
//             franka::RobotState state = current_state_;
//             franka::Duration period(0.001);
//             auto motion = callback(state, period);
//             if (motion.motion_finished)
//             {
//                 break;
//             }
//             std::this_thread::sleep_for(std::chrono::milliseconds(1));
//         }
//         zlc::info("FakeFrankaRobot: control loop stopped.");
//         isOnControlLoop = false;
//     }

//     template <typename ControllerCallback>
//     void control(ControllerCallback callback, franka::ControllerMode mode, bool limit_rate = true,
//                  double cutoff_frequency = (100.0))
//     {
//         control(callback, limit_rate, cutoff_frequency);
//     }

//     template <typename MotionGeneratorCallback, typename TorqueControlCallback>
//     void control(MotionGeneratorCallback motion_callback, TorqueControlCallback torque_callback,
//                  bool limit_rate = true, double cutoff_frequency = (100.0))
//     {
//         zlc::info("FakeFrankaRobot: control loop started.");
//         isOnControlLoop = true;
//         while (isOnControlLoop)
//         {
//             franka::RobotState state = current_state_;
//             franka::Duration period(0.001);
//             auto motion = motion_callback(state, period);
//             auto torque = torque_callback(state, period);
//             if (motion.motion_finished || torque.motion_finished)
//             {
//                 break;
//             }
//             std::this_thread::sleep_for(std::chrono::milliseconds(1));
//         }
//         zlc::info("FakeFrankaRobot: control loop stopped.");
//         isOnControlLoop = false;
//     }
//     // Add other necessary fake methods as needed for testing

//     franka::RobotState readOnce()
//     {
//         if (isOnControlLoop)
//         {
//             throw std::runtime_error("Cannot read state while on control loop.");
//         }
//         return current_state_;
//     }

//     FakeFrankaModel loadModel()
//     {
//         return FakeFrankaModel();
//     }

//   private:
//     std::atomic<bool> isOnControlLoop = false;
//     franka::RobotState current_state_;
// };