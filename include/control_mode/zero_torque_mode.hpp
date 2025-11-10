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
    ZeroTorqueMode();
    ~ZeroTorqueMode() override;

    //void initialize(const franka::RobotState& initial_state) override;
    void controlLoop() override;
    void start();
    void stop() override;
    const std::string& getModeName() const override;
private:
    void writeCommand(const std::vector<uint8_t>& data) override;
};
