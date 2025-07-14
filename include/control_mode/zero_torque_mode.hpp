#ifndef ZERO_TORQUE_MODE_HPP
#define ZERO_TORQUE_MODE_HPP

#include "abstract_control_mode.h"
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/exception.h>
#include <memory>//for std::shared_ptr
#include <mutex>
#include <franka/exception.h>
#include <iostream>

class ZeroTorqueMode : public AbstractControlMode {
public:
    ZeroTorqueMode() = default;
    ~ZeroTorqueMode() override = default;

    //void initialize(const franka::RobotState& initial_state) override;
    void start() override;
    void stop() override;
    int getModeID() const override;
};

#endif // ZERO_TORQUE_MODE_HPP