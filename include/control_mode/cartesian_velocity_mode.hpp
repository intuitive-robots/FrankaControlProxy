#pragma once
#include "abstract_control_mode.hpp"
#include <franka/robot_state.h>
#include <functional>
#include "utils/atomic_double_buffer.hpp"

class CartesianVelocityMode : public AbstractControlMode {
public:
    CartesianVelocityMode();
    ~CartesianVelocityMode() override;

    protocol::ModeID getModeID() const override;

private:
    AtomicDoubleBuffer<franka::CartesianVelocities> desired_velocities_;
    void controlLoop() override;
    void writeCommand(const protocol::ByteView& data) override;
    void writeZeroCommand() override;
};
