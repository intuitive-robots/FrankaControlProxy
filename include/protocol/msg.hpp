// #include <zerolancom/zerolancom.hpp>
#include <vector>
#include <msgpack.hpp>

struct FrankaRobotState
{
    std::vector<double> q;
    std::vector<double> q_d;
    std::vector<double> dq;
    std::vector<double> dq_d;
    std::vector<double> tau_ext_hat_filtered;
    MSGPACK_DEFINE_MAP(q, q_d, dq, dq_d, tau_ext_hat_filtered);
};

struct 
{
    /* data */
};
