// #pragma once

// #include <array>
// #include <atomic>
// #include <msgpack.hpp>

// #include "control_mode/abstract_control_mode.hpp"

// struct CartesianPoseCommand
// {
//     std::array<double, 7> pose;
//     MSGPACK_DEFINE_MAP(pose);
// };

// struct CartesianPoseConfig
// {
//     std::string command_topic{"FRANKA_CARTESIAN_POSE_CMD"};
//     std::array<double, 6> k_gains{{150.0, 150.0, 150.0, 20.0, 20.0, 20.0}};
//     std::array<double, 6> d_gains{{20.0, 20.0, 20.0, 2.0, 2.0, 2.0}};

//     CartesianPoseConfig()
//     {
//         fromFile("config/controller/CartesianPoseController.yaml");
//     }

//     void fromFile(const std::string& controller_config_path)
//     {
//         ConfigFileReader reader(controller_config_path);
//         command_topic = reader.getValue<std::string>("command_topic", command_topic);
//         k_gains = reader.getArray<6>("k_gains", k_gains);
//         d_gains = reader.getArray<6>("d_gains", d_gains);
//     }
// };

// /**
//  * @brief Cartesian pose control mode.
//  *
//  * Controls the end-effector position and orientation in Cartesian space
//  * using a simple Cartesian impedance law and Pinocchio model.
//  */
// class CartesianPoseMode : public AbstractControlMode
// {
//   public:
//     CartesianPoseMode();
//     ~CartesianPoseMode() override;

//   private:
//     void initController() override;
//     franka::Torques controlLoop(const franka::RobotState& robot_state,
//                                 franka::Duration duration) override;
//     void writeCommand(const CartesianPoseCommand& cmd);

//     AtomicDoubleBuffer<std::array<double, 7>> desired_pose_;
//     std::atomic<bool> has_target_{false};
//     CartesianPoseConfig config_;
// };
