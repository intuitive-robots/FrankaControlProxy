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
    const std::string& getModeName() override;
private:
    AtomicDoubleBuffer<franka::JointVelocities> desired_velocities_;
    void writeCommand(const std::vector<uint8_t>& data) override;

};
