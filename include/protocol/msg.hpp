// #include <zerolancom/zerolancom.hpp>
#include <vector>
#include <msgpack.hpp>
#include "grasp_command.hpp"
#include "request_result.hpp"

struct FrankaRobotState
{
    uint32_t time_ms;
    std::vector<double> O_T_EE;
    std::vector<double> O_T_EE_d;
    std::vector<double> q;
    std::vector<double> q_d;
    std::vector<double> dq;
    std::vector<double> dq_d;
    std::vector<double> tau_ext_hat_filtered;
    std::vector<double> O_F_ext_hat_K;
    std::vector<double> K_F_ext_hat_K;
    MSGPACK_DEFINE_MAP(time_ms, O_T_EE, O_T_EE_d, q, q_d, dq, dq_d, tau_ext_hat_filtered, O_F_ext_hat_K, K_F_ext_hat_K);
};

struct FrankaGripperState
{
    double width;
    double max_width;
    bool is_grasped;
    uint16_t temperature;
    MSGPACK_DEFINE_MAP(width, max_width, is_grasped, temperature);
};

struct FrankaControlCommand
{
    int mode_id;
    std::vector<double> target_values;
    MSGPACK_DEFINE_MAP(mode_id, target_values);
};

struct GraspCommandMsg
{
    double width;
    double speed;
    double force;
    MSGPACK_DEFINE_MAP(width, speed, force);
};

struct FrankaResponseMsg
{
    std::string code;
    std::vector<uint8_t> payload;
    MSGPACK_DEFINE_MAP(code, payload);
};

