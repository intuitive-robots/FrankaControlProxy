#ifndef CARTESIAN_VELOCITY_MODE_HPP
#define CARTESIAN_VELOCITY_MODE_HPP

#include "abstract_control_mode.hpp"
#include <franka/robot_state.h>
#include <functional>
#include "utils/AtomicDoubleBuffer.hpp"

class CartesianVelocityMode : public AbstractControlMode {
public:
    CartesianVelocityMode();
    ~CartesianVelocityMode() override;

    void start() override;
    void stop() override;
    int getModeID() const override;

private:
    AtomicDoubleBuffer<franka::CartesianVelocities> desired_velocities_;
};

#endif // CARTESIAN_VELOCITY_MODE_HPP
