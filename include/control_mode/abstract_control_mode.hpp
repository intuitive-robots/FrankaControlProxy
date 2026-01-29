#pragma once

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
    std::string command_topic;

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

    virtual void initController(FrankaPanda& robot, PandaPinocchioModel& pinocchio_model,
                                AtomicDoubleBuffer<franka::RobotState>& state_buffer);
    void startControl();
    void stopControl();
    const std::string getModeName();
    void controlTask();
    bool moveToJointPosition(const std::array<double, NUM_DOFS>& target_q,
                             double max_velocity = 0.1, double tolerance = 1e-2);
  protected:
    AbstractControlMode(const SafetyLimitConfig& safety_config) : safety_config_(safety_config) {}
    FrankaPanda* robot_;
    PandaPinocchioModel* pinocchio_model_;
    AtomicDoubleBuffer<franka::RobotState>* state_buffer_;

    bool is_running_ = false;
    std::string controller_name{"AbstractControlMode"};
    bool tryRecovery(int max_attempts = 3);

    virtual franka::Torques controlLoop(const franka::RobotState& robot_state,
                                        franka::Duration duration) = 0;
    std::thread control_thread_;
    const SafetyLimitConfig& safety_config_;

  private:
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
class MotionGenerator 
{
  public:
    /**
     * Creates a new MotionGenerator instance for a target q.
     *
     * @param[in] speed_factor General speed factor in range [0, 1].
     * @param[in] q_goal Target joint positions.
     */
    MotionGenerator(double speed_factor, const std::array<double, 7>& q_goal);

      /**
       * Sends joint position calculations
       *
       * @param[in] robot_state Current state of the robot.
       * @param[in] period Duration of execution.
       *
       * @return Joint positions for use inside a control loop.
       */
      franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);

    private:
      using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
      using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

      bool calculateDesiredValues(double time, Vector7d* delta_q_d) const;
      void calculateSynchronizedValues();

      static constexpr double kDeltaQMotionFinished = 1e-6;
      Vector7d q_goal_;

      Vector7d q_start_;
      Vector7d delta_q_;

      Vector7d dq_max_sync_;
      Vector7d t_1_sync_;
      Vector7d t_2_sync_;
      Vector7d t_f_sync_;
      Vector7d q_1_;

      double time_ = 0.0;

      Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
      Vector7d ddq_max_start_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
      Vector7d ddq_max_goal_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
};
