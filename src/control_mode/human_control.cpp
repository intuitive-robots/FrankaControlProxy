#include "control_mode/human_control.hpp"

franka::Torques HumanControlMode::controlLoop(const franka::RobotState& /*robot_state*/,
                                              franka::Duration /*duration*/)
{
    std::array<double, NUM_DOFS> tau_cmd{};
    tau_cmd.fill(0.0);
    return franka::Torques(tau_cmd);
}
