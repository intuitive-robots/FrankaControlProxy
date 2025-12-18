// #include <zerolancom/zerolancom.hpp>
#include <vector>
#include <msgpack.hpp>
#include <franka/robot_state.h>

struct FrankaRobotState
{
    std::vector<double> q;
    std::vector<double> q_d;
    std::vector<double> dq;
    std::vector<double> dq_d;
    std::vector<double> tau_ext_hat_filtered;
    MSGPACK_DEFINE_MAP(q, q_d, dq, dq_d, tau_ext_hat_filtered);

    FrankaRobotState(const franka::RobotState& state) 
        : q(state.q.begin(), state.q.end()),
          q_d(state.q_d.begin(), state.q_d.end()),
          dq(state.dq.begin(), state.dq.end()),
          dq_d(state.dq_d.begin(), state.dq_d.end()),
          tau_ext_hat_filtered(state.tau_ext_hat_filtered.begin(), state.tau_ext_hat_filtered.end())
    {};

};

struct 
{
    /* data */
};
