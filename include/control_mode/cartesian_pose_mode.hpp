#pragma once
#include "abstract_control_mode.hpp"
#include <franka/robot_state.h>
#include <functional>
#include "utils/atomic_double_buffer.hpp"

/**
 * @brief Cartesian position control mode.
 *
 * Controls the end-effector position and orientation in Cartesian space
 * using franka::CartesianPose commands.
 */
class CartesianPoseMode : public AbstractControlMode {
public:
    CartesianPoseMode();
    ~CartesianPoseMode() override;

    protocol::ModeID getModeID() const override;

private:
    AtomicDoubleBuffer<franka::CartesianPose> desired_pose_;
    void controlLoop() override;
    void writeCommand(const protocol::ByteView& data) override;
};
