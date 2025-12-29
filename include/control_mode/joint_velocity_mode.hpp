#pragma once
#include "abstract_control_mode.hpp"
#include <franka/robot_state.h>
#include <array>
#include <functional>
#include "utils/atomic_double_buffer.hpp"

class JointVelocityMode : public AbstractControlMode {
public:
    JointVelocityMode();
    ~JointVelocityMode() override;
    void controlLoop() override;
    protocol::ControlModeID getControlModeID() const override;
private:
    AtomicDoubleBuffer<franka::JointVelocities> desired_velocities_;
    void writeCommand(const protocol::ByteView& data) override;
    void writeZeroCommand() override;
};
