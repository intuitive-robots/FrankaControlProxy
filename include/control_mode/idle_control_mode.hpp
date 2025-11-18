#pragma once
#include "abstract_control_mode.hpp"
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/exception.h>
#include <memory>
#include <mutex>
#include <franka/exception.h>
#include <iostream>


class IdleControlMode : public AbstractControlMode {
public:
    IdleControlMode();
    ~IdleControlMode() override ;
    void start() override;
    //void initialize(const franka::RobotState& initial_state) override;
    protocol::ModeID getModeID() const override;

private:
    void controlLoop() override;
    void writeCommand(const protocol::ByteView& data) override;
    void writeZeroCommand() override;
};