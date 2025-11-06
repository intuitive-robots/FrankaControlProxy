#ifndef JOINT_VELOCITY_MODE_HPP
#define JOINT_VELOCITY_MODE_HPP
#include "abstract_control_mode.hpp"
#include <franka/robot_state.h>
#include <array>
#include <functional>
#include "utils/AtomicDoubleBuffer.hpp"

class JointVelocityMode : public AbstractControlMode {
public:
    JointVelocityMode();
    ~JointVelocityMode() override;
    void controlLoop() override;
    void stop() override;
    int getModeID() const override;
private:
    AtomicDoubleBuffer<franka::JointVelocities> desired_velocities_;

};
#endif // JOINT_VELOCITY_MODE_HPP