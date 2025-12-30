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

// class HumanControlMode : public AbstractControlMode {
// public:
//     HumanControlMode();
//     ~HumanControlMode() override;

//     //void initialize(const franka::RobotState& initial_state) override;
//     void controlLoop() override;
//     void start();
//     protocol::ControlModeID getControlModeID() const override;
// private:
//     void writeCommand(const protocol::ByteView& data) override;
//     void writeZeroCommand() override;
// };
