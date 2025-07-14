#include "control_mode/control_mode_registry.h"
#include "control_mode/idle_control_mode.hpp"
#include "control_mode/zero_torque_mode.hpp"
#include "control_mode/joint_pd_test_mode.hpp"


void registerAllControlModes(FrankaProxy& proxy) {
    proxy.registerControlMode("idle", std::make_unique<IdleControlMode>());
    proxy.registerControlMode("zero_torque", std::make_unique<ZeroTorqueMode>());
    proxy.registerControlMode("joint_pd", std::make_unique<JointPDControlMode>());
}