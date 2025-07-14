#ifndef JOINT_PD_CONTROL_TEST_HPP
#define JOINT_PD_CONTROL_TEST_HPP

#include "abstract_control_mode.h"
#include <franka/robot_state.h>
#include <array>
#include <functional>

class JointPDControlMode : public AbstractControlMode {
public:
    JointPDControlMode();
    ~JointPDControlMode() override;

    void start() override;
    void stop() override;
    int getModeID() const override;
};

#endif // JOINT_PD_CONTROL_TEST_HPP
