#pragma once
#include "abstract_control_mode.hpp"
#include <franka/robot_state.h>
#include <array>
#include <functional>
#include "utils/atomic_double_buffer.hpp"

class JointPositionMode : public AbstractControlMode {
public:
    JointPositionMode();
    ~JointPositionMode() override;
    void controlLoop() override;
    protocol::ModeID getModeID() const override;
private:
    AtomicDoubleBuffer<franka::JointPositions> desired_positions_;
    void writeCommand(const protocol::ByteView& data) override;
    void writeZeroCommand() override;
};
