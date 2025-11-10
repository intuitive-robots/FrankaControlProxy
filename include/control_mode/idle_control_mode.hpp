#ifndef IDLE_CONTROL_MODE_H
#define IDLE_CONTROL_MODE_H

#include "abstract_control_mode.h"
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/exception.h>
#include <memory>//for std::shared_ptr
#include <mutex>
#include <franka/exception.h>
#include <iostream>


//todo: need to reconstruct
class IdleControlMode : public AbstractControlMode {
public:
    IdleControlMode() = default;
    ~IdleControlMode() override = default;

    //void initialize(const franka::RobotState& initial_state) override;
    void start() override;
    void stop() override;
    int getModeID() const override;
};

    
#endif // IDLE_CONTROL_MODE_H