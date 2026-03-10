// #pragma once
// #include <array>
// #include <msgpack.hpp>

// #include "control_mode/abstract_control_mode.hpp"
// #include "protocol/control_command.hpp"
// #include "utils/linear_trajInterpolator.hpp"

// struct OSCConfig : public ControllerConfig
// {
//     std::string command_topic{"FRANKA_HYBRID_JOINT_IMPEDANCE_CMD"};
//     Eigen::Matrix<double, 3, 3> Kp_p{Eigen::Matrix<double, 3, 3>::Zero()};
//     Eigen::Matrix<double, 3, 3> Kp_r{Eigen::Matrix<double, 3, 3>::Zero()};
//     Eigen::Matrix<double, 3, 3> Kd_p{Eigen::Matrix<double, 3, 3>::Zero()};
//     Eigen::Matrix<double, 3, 3> Kd_r{Eigen::Matrix<double, 3, 3>::Zero()};
//     Eigen::Matrix<double, 7, 1> static_q_task{Eigen::Matrix<double, 7, 1>::Zero()};
//     Eigen::Matrix<double, 7, 1> residual_mass_vec{Eigen::Matrix<double, 7, 1>::Zero()};
//     Eigen::Array<double, 7, 1> joint_max{Eigen::Array<double, 7, 1>::Zero()};
//     Eigen::Array<double, 7, 1> joint_min{Eigen::Array<double, 7, 1>::Zero()};
//     Eigen::Array<double, 7, 1> avoidance_weights{Eigen::Array<double, 7, 1>::Zero()};
//     bool ignore_gravity{true};
//     OSCConfig() = default;

//     void fromFile(const std::string& controller_config_path) override
//     {
//         ConfigFileReader reader(controller_config_path);
//         readBaseConfig(reader);
//         command_topic = reader.getValue<std::string>("command_topic");

//         // Read Kp_p and Kp_r diagonal values
//         const std::array<double, 3> Kp_p_vals = reader.getArray<double, 3>("Kp_p");
//         Kp_p = Eigen::Vector3d::Map(Kp_p_vals.data()).asDiagonal();
//         const std::array<double, 3> Kp_r_vals = reader.getArray<double, 3>("Kp_r");
//         Kp_r = Eigen::Vector3d::Map(Kp_r_vals.data()).asDiagonal();

//         // Compute Kd from Kp (critically damped)
//         Kd_p = Kp_p.cwiseSqrt() * 2.0;
//         Kd_r = Kp_r.cwiseSqrt() * 2.0;

//         // Read 7-element vectors
//         const std::array<double, 7> static_q_task_vals =
//             reader.getArray<double, 7>("static_q_task");
//         static_q_task = Eigen::Matrix<double, 7, 1>::Map(static_q_task_vals.data());

//         const std::array<double, 7> residual_mass_vec_vals =
//             reader.getArray<double, 7>("residual_mass_vec");
//         residual_mass_vec = Eigen::Matrix<double, 7, 1>::Map(residual_mass_vec_vals.data());

//         const std::array<double, 7> joint_max_vals = reader.getArray<double, 7>("joint_max");
//         joint_max = Eigen::Array<double, 7, 1>::Map(joint_max_vals.data());

//         const std::array<double, 7> joint_min_vals = reader.getArray<double, 7>("joint_min");
//         joint_min = Eigen::Array<double, 7, 1>::Map(joint_min_vals.data());

//         const std::array<double, 7> avoidance_weights_vals =
//             reader.getArray<double, 7>("avoidance_weights");
//         avoidance_weights = Eigen::Array<double, 7, 1>::Map(avoidance_weights_vals.data());

//         ignore_gravity = reader.getValue<bool>("ignore_gravity");
//     }
// };

// class OSCController : public AbstractControlMode
// {
//   public:
//     explicit OSCController(const SafetyLimitConfig& safety_config, const std::string& robot_name)
//         : AbstractControlMode(safety_config, robot_name)
//     {
//         controller_name = "OSC";
//         traj_interpolator = LinearPoseTrajInterpolator();
//     }
//     ~OSCController() override = default;

//   private:
//     void startControl() override;
//     franka::Torques controlLoop(const franka::RobotState& robot_state,
//                                 franka::Duration duration) override;
//     void writeCommand(const CartesianPoseCommand& cmd);
//     LinearPoseTrajInterpolator traj_interpolator;
//     double controller_time{0.};
//     OSCConfig config_;
// };

// void PInverse(const Eigen::MatrixXd& M, Eigen::MatrixXd& M_inv, double epsilon = 0.00025);

// void TorqueSafetyGuardFn(std::array<double, 7>& tau_d_array, double min_torque, double max_torque);