#pragma once

#include "abstract_control_mode.hpp"
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
    void controlLoop() override;
    void stop() override;
    int getModeID() const override;
};
