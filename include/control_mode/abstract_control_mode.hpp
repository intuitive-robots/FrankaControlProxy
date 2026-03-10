#pragma once

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <thread>
#include <zerolancom/zerolancom.hpp>

#include "utils/atomic_double_buffer.hpp"
#include "utils/config_file_reader.hpp"
#include "utils/robot_model.hpp"
#include "utils/robot_utils.hpp"

struct AbstractConfig
{
    AbstractConfig() = default;
    ~AbstractConfig() = default;
    virtual void fromFile(const std::string& controller_config_path) = 0;
};

struct ControllerConfig : public AbstractConfig
{
    // communication
    std::string controller_name;

    ControllerConfig() = default;

    // void fromFile(const std::string& controller_config_path) override;
    void readBaseConfig(const ConfigFileReader& reader);
};

struct SafetyLimitConfig : public AbstractConfig
{
    // control settings
    bool limit_rate;
    double lpf_cutoff_freq;

    // cartesian limits
    std::array<double, 3> cartesian_pos_upper_limits;
    std::array<double, 3> cartesian_pos_lower_limits;

    // joint limits
    std::array<double, NUM_DOFS> joint_pos_upper_limits;
    std::array<double, NUM_DOFS> joint_pos_lower_limits;
    std::array<double, NUM_DOFS> joint_vel_upper_limits;
    std::array<double, NUM_DOFS> joint_vel_lower_limits;
    std::array<double, NUM_DOFS> joint_torques_limits;

    // safety controller
    double margin_joint_pos;
    double margin_joint_vel;
    double k_joint_pos;
    double k_joint_vel;

    SafetyLimitConfig() = default;
    void fromFile(const std::string& controller_config_path) override;
};

class AbstractControlMode
{
  public:
    virtual ~AbstractControlMode() = default;

    void initController(FrankaPanda& robot, PandaPinocchioModel& pinocchio_model,
                                AtomicDoubleBuffer<franka::RobotState>& state_buffer,
                                const SafetyLimitConfig& safety_config);
    virtual void startControl();
    virtual void stopControl();
    const std::string getModeName();
    virtual void controlTask();
    bool moveToJointPosition(const std::array<double, NUM_DOFS>& target_q,
                             double max_velocity = 0.1, double tolerance = 1e-2);
    bool moveToCartesianPose(const Eigen::Vector3d& target_position,
                             const Eigen::Quaterniond& target_orientation,
                             double max_velocity = 0.5, double tolerance = 1e-3);

  protected:
    AbstractControlMode() { controller_name = "AbstractControlMode"; };
    FrankaPanda* robot_;
    PandaPinocchioModel* pinocchio_model_;
    AtomicDoubleBuffer<franka::RobotState>* state_buffer_;

    bool is_running_ = false;
    std::string controller_name;
    bool tryRecovery(int max_attempts = 3);

    virtual franka::Torques controlLoop(const franka::RobotState& robot_state,
                                        franka::Duration duration)
    {
        return franka::Torques{};
    }
    std::thread control_thread_;

  private:
    const SafetyLimitConfig* safety_config_;
    void checkStateLimits(const franka::RobotState& robot_state, franka::Torques& torque_out,
                          const SafetyLimitConfig& safety_config_);
    void postprocessTorques(franka::Torques& torque_applied,
                            const std::array<double, NUM_DOFS>& torque_limits);
    template <std::size_t N>
    void computeSafetyReflex(std::array<double, N> values, std::array<double, N> lower_limit,
                             std::array<double, N> upper_limit, std::array<double, N>& torques_out,
                             double margin, double k);
    std::unordered_map<std::string, bool> active_constraints_map_;
};
