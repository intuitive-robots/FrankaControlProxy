// #pragma once
// #include "control_mode/abstract_control_mode.hpp"



// /**
//  * @brief Cartesian position control mode.
//  *
//  * Controls the end-effector position and orientation in Cartesian space
//  * using franka::CartesianPose commands.
//  */
// class CartesianPoseMode : public AbstractControlMode {
// public:
//     CartesianPoseMode();
//     ~CartesianPoseMode() override;

//     // protocol::ControlModeID getControlModeID() const override;

// private:
//     AtomicDoubleBuffer<franka::CartesianPose> desired_pose_;
//     // void controlLoop() override;
//     // void writeCommand(const CartesianVelocityCommand& cmd);
//     // void writeZeroCommand() override;
// };
