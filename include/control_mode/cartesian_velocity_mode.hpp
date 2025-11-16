#pragma once
#include "abstract_control_mode.hpp"
#include <franka/robot_state.h>
#include <functional>
#include "utils/atomic_double_buffer.hpp"

class CartesianVelocityMode : public AbstractControlMode {
public:
    CartesianVelocityMode();
    ~CartesianVelocityMode() override;

    void start();
    const std::string& getModeName() override;

private:
    AtomicDoubleBuffer<franka::CartesianVelocities> desired_velocities_;
    void controlLoop() override;
    void writeCommand(const std::vector<uint8_t>& data) override;
};
