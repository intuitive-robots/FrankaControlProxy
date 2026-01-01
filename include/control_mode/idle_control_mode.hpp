#pragma once
#include "abstract_control_mode.hpp"
#include <thread>



class IdleControlMode : public AbstractControlMode {
public:
    IdleControlMode();
    ~IdleControlMode() override ;
private:
    void initController() override;
    void controlLoop() override;
};