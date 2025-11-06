#pragma once
#include "abstract_control_mode.hpp"
#include <franka/robot_state.h>
#include <array>
#include <functional>
#include "utils/AtomicDoubleBuffer.hpp"

class JointPositionMode : public AbstractControlMode {
public:
    JointPositionMode();
    ~JointPositionMode() override;
    void start() override;
    void stop() override;
    int getModeID() const override;
private:
    AtomicDoubleBuffer<franka::JointPositions> desired_positions_;

};
