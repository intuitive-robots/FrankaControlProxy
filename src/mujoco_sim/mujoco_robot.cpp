#include "mujoco_sim/mujoco_robot.hpp"

#include <franka/exception.h>
#include <mujoco/mujoco.h>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "mujoco_sim/mujoco_panda_env.hpp"

namespace
{
    franka::Duration toDuration(std::chrono::nanoseconds dt)
    {
        return franka::Duration(static_cast<uint64_t>(dt.count()));
    }
} // namespace

MujocoRobot::MujocoRobot(const std::string&, const std::string& env_config_path)
{
    // Load configuration
    config_.fromFile(env_config_path);
    control_period_ = std::chrono::microseconds(1000000 / config_.control_rate);

    env_ = std::make_unique<MujocoPandaEnv>("./models/franka_emika_panda/scene.xml", config_);
    model_ = std::make_unique<PandaPinocchioModel>("./models/franka_emika_panda/panda_arm.urdf",
                                                   "panda_link8");
    env_->start();
    env_->refreshRobotState(current_state_);

    if (config_.enable_viewer)
    {
        viewer_ = std::make_unique<MujocoViewer>(env_.get());
        viewer_->start();
    }
}

MujocoRobot::~MujocoRobot() noexcept
{
    stop();
}

void MujocoRobot::control(
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> control_callback,
    bool /*limit_rate*/, double /*cutoff_frequency*/)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    if (running_)
    {
        throw franka::InvalidOperationException(
            "MujocoRobot::control: A control loop is already running.");
    }

    running_ = true;
    using clock = std::chrono::steady_clock;
    auto next_tick = clock::now();
    auto last_tick = next_tick;

    while (running_)
    {
        const auto now = clock::now();
        auto dt_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_tick);
        last_tick = now;
        franka::Torques torques = control_callback(current_state_, toDuration(dt_ns));
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_.tau_J_d = torques.tau_J;
        }
        env_->nextStep(torques, current_state_);
        if (torques.motion_finished)
        {
            running_ = false;
            break;
        }

        next_tick += control_period_;
        std::this_thread::sleep_until(next_tick);
    }
}

void MujocoRobot::read(std::function<bool(const franka::RobotState&)> read_callback)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    if (running_)
    {
        throw franka::InvalidOperationException(
            "MujocoRobot::read: A control loop is already running.");
    }

    running_ = true;
    auto next_tick = std::chrono::steady_clock::now();
    franka::Torques zero_torques{};
    zero_torques.tau_J.fill(0.0);

    while (running_)
    {
        if (!read_callback(current_state_))
        {
            running_ = false;
            break;
        }
        next_tick += control_period_;
        std::this_thread::sleep_until(next_tick);
    }
}

franka::RobotState MujocoRobot::readOnce()
{
    if (running_)
    {
        throw franka::InvalidOperationException(
            "MujocoRobot::readOnce: A control loop is already running.");
    }
    return current_state_;
}

void MujocoRobot::setCollisionBehavior(
    const std::array<double, 7>& /*lower_torque_thresholds_acceleration*/,
    const std::array<double, 7>& /*upper_torque_thresholds_acceleration*/,
    const std::array<double, 7>& /*lower_torque_thresholds_nominal*/,
    const std::array<double, 7>& /*upper_torque_thresholds_nominal*/,
    const std::array<double, 6>& /*lower_force_thresholds_acceleration*/,
    const std::array<double, 6>& /*upper_force_thresholds_acceleration*/,
    const std::array<double, 6>& /*lower_force_thresholds_nominal*/,
    const std::array<double, 6>& /*upper_force_thresholds_nominal*/)
{
    // No-op: collision behavior is not modeled in this MuJoCo wrapper.
}

void MujocoRobot::setCollisionBehavior(const std::array<double, 7>& /*lower_torque_thresholds*/,
                                       const std::array<double, 7>& /*upper_torque_thresholds*/,
                                       const std::array<double, 6>& /*lower_force_thresholds*/,
                                       const std::array<double, 6>& /*upper_force_thresholds*/)
{
    // No-op: collision behavior is not modeled in this MuJoCo wrapper.
}

void MujocoRobot::setJointImpedance(const std::array<double, 7>& /*K_theta*/)
{
    // No-op: joint impedance is a robot firmware setting, not applicable in MuJoCo simulation.
}

void MujocoRobot::setCartesianImpedance(const std::array<double, 6>& /*K_x*/)
{
    // No-op: Cartesian impedance is a robot firmware setting, not applicable in MuJoCo simulation.
}

void MujocoRobot::automaticErrorRecovery()
{
    // No-op for simulation.
}

void MujocoRobot::stop()
{
    running_ = false;
    env_->stop();
    viewer_->stop();
}

// ============================================================================
// Control conversion helpers
// ============================================================================

franka::Torques MujocoRobot::jointPositionToTorque(const franka::JointPositions& desired_positions,
                                                   const franka::RobotState& state)
{
    std::array<double, 7> tau_J{};
    // Get gravity compensation from MuJoCo
    const auto* data = env_->getData();

    for (size_t i = 0; i < 7; i++)
    {
        double pos_error = desired_positions.q[i] - state.q[i];
        double vel_error = 0.0 - state.dq[i]; // Target velocity is zero for position control
        tau_J[i] = kDefaultStiffness[i] * pos_error + kDefaultDamping[i] * vel_error;
        // Add gravity compensation
        tau_J[i] += data->qfrc_bias[i];
    }
    franka::Torques torques(tau_J);
    torques.motion_finished = desired_positions.motion_finished;
    return torques;
}

franka::Torques MujocoRobot::jointVelocityToTorque(
    const franka::JointVelocities& desired_velocities, const franka::RobotState& state)
{
    std::array<double, 7> tau_J{};
    // Get gravity compensation from MuJoCo
    const auto* data = env_->getData();

    for (size_t i = 0; i < 7; i++)
    {
        // Pure velocity damping control
        double vel_error = desired_velocities.dq[i] - state.dq[i];
        tau_J[i] = kDefaultDamping[i] * vel_error;
        // Add gravity compensation
        tau_J[i] += data->qfrc_bias[i];
    }
    franka::Torques torques(tau_J);
    torques.motion_finished = desired_velocities.motion_finished;
    return torques;
}

std::array<double, 7> MujocoRobot::cartesianPoseToJointPosition(
    const franka::CartesianPose& desired_pose, const franka::RobotState& state)
{
    // Extract target position and orientation from 4x4 matrix (column-major)
    Eigen::Vector3d target_pos(desired_pose.O_T_EE[12], desired_pose.O_T_EE[13],
                               desired_pose.O_T_EE[14]);
    Eigen::Matrix3d target_rot;
    target_rot << desired_pose.O_T_EE[0], desired_pose.O_T_EE[4], desired_pose.O_T_EE[8],
        desired_pose.O_T_EE[1], desired_pose.O_T_EE[5], desired_pose.O_T_EE[9],
        desired_pose.O_T_EE[2], desired_pose.O_T_EE[6], desired_pose.O_T_EE[10];
    Eigen::Quaterniond target_quat(target_rot);

    // Start from current joint positions
    JointPosition q;
    for (size_t i = 0; i < 7; i++)
    {
        q[i] = state.q[i];
    }

    // Iterative IK using Jacobian pseudoinverse
    for (int iter = 0; iter < kIKMaxIterations; iter++)
    {
        // Compute current FK
        PoseQuat current_pose = model_->forwardKinematics(q);
        Eigen::Vector3d current_pos(current_pose[0], current_pose[1], current_pose[2]);
        Eigen::Quaterniond current_quat(current_pose[6], current_pose[3], current_pose[4],
                                        current_pose[5]);

        // Compute position error
        Eigen::Vector3d pos_error = target_pos - current_pos;

        // Compute orientation error (using quaternion error)
        Eigen::Quaterniond quat_error = target_quat * current_quat.inverse();
        if (quat_error.w() < 0.0)
        {
            quat_error.coeffs() *= -1.0; // Ensure shortest path
        }
        Eigen::Vector3d rot_error = 2.0 * quat_error.vec(); // Approximate axis-angle

        // Check convergence
        double error_norm = std::sqrt(pos_error.squaredNorm() + rot_error.squaredNorm());
        if (error_norm < kIKErrorThreshold)
        {
            break;
        }

        // Compute Jacobian
        JocobianMatrix J = model_->computeJacobian(q);

        // Compute 6D error vector
        Eigen::Matrix<double, 6, 1> dx;
        dx << pos_error, rot_error;

        // Compute Jacobian pseudoinverse: J† = J^T (J J^T)^{-1}
        Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
        Eigen::Matrix<double, 6, 6> JJt_damped =
            JJt + 1e-6 * Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 7, 6> J_pinv = J.transpose() * JJt_damped.inverse();

        // Compute joint delta
        JointPosition dq = J_pinv * dx;

        // Update joint positions
        q += kIKStepSize * dq;
    }

    std::array<double, 7> result;
    for (size_t i = 0; i < 7; i++)
    {
        result[i] = q[i];
    }
    return result;
}

std::array<double, 7> MujocoRobot::cartesianVelocityToJointVelocity(
    const franka::CartesianVelocities& desired_velocities, const franka::RobotState& state)
{
    // Get current joint positions
    JointPosition q;
    for (size_t i = 0; i < 7; i++)
    {
        q[i] = state.q[i];
    }

    // Compute Jacobian at current configuration
    JocobianMatrix J = model_->computeJacobian(q);

    // Compute 6D Cartesian velocity vector
    Eigen::Matrix<double, 6, 1> dx;
    dx << desired_velocities.O_dP_EE[0], desired_velocities.O_dP_EE[1],
        desired_velocities.O_dP_EE[2], desired_velocities.O_dP_EE[3], desired_velocities.O_dP_EE[4],
        desired_velocities.O_dP_EE[5];

    // Compute Jacobian pseudoinverse: J† = J^T (J J^T)^{-1}
    Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    Eigen::Matrix<double, 6, 6> JJt_damped = JJt + 1e-6 * Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 7, 6> J_pinv = J.transpose() * JJt_damped.inverse();

    // Compute joint velocities: dq = J† * dx
    JointVelocity dq = J_pinv * dx;

    std::array<double, 7> result;
    for (size_t i = 0; i < 7; i++)
    {
        result[i] = dq[i];
    }
    return result;
}

// ============================================================================
// Control overloads for different command types
// ============================================================================

void MujocoRobot::control(
    std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
        motion_generator_callback,
    bool /*limit_rate*/, double /*cutoff_frequency*/)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    if (running_)
    {
        throw franka::InvalidOperationException(
            "MujocoRobot::control: A control loop is already running.");
    }

    running_ = true;
    using clock = std::chrono::steady_clock;
    auto next_tick = clock::now();
    auto last_tick = next_tick;

    while (running_)
    {
        const auto now = clock::now();
        auto dt_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_tick);
        last_tick = now;

        franka::JointPositions desired_positions =
            motion_generator_callback(current_state_, toDuration(dt_ns));
        franka::Torques torques = jointPositionToTorque(desired_positions, current_state_);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_.tau_J_d = torques.tau_J;
        }
        env_->nextStep(torques, current_state_);

        if (desired_positions.motion_finished)
        {
            running_ = false;
            break;
        }

        next_tick += control_period_;
        std::this_thread::sleep_until(next_tick);
    }
}

void MujocoRobot::control(
    std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)>
        motion_generator_callback,
    bool /*limit_rate*/, double /*cutoff_frequency*/)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    if (running_)
    {
        throw franka::InvalidOperationException(
            "MujocoRobot::control: A control loop is already running.");
    }

    running_ = true;
    using clock = std::chrono::steady_clock;
    auto next_tick = clock::now();
    auto last_tick = next_tick;

    while (running_)
    {
        const auto now = clock::now();
        auto dt_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_tick);
        last_tick = now;

        franka::JointVelocities desired_velocities =
            motion_generator_callback(current_state_, toDuration(dt_ns));
        franka::Torques torques = jointVelocityToTorque(desired_velocities, current_state_);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_.tau_J_d = torques.tau_J;
        }
        env_->nextStep(torques, current_state_);

        if (desired_velocities.motion_finished)
        {
            running_ = false;
            break;
        }

        next_tick += control_period_;
        std::this_thread::sleep_until(next_tick);
    }
}

void MujocoRobot::control(
    std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)>
        motion_generator_callback,
    bool /*limit_rate*/, double /*cutoff_frequency*/)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    if (running_)
    {
        throw franka::InvalidOperationException(
            "MujocoRobot::control: A control loop is already running.");
    }

    running_ = true;
    using clock = std::chrono::steady_clock;
    auto next_tick = clock::now();
    auto last_tick = next_tick;

    while (running_)
    {
        const auto now = clock::now();
        auto dt_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_tick);
        last_tick = now;

        franka::CartesianPose desired_pose =
            motion_generator_callback(current_state_, toDuration(dt_ns));

        // Convert Cartesian pose to joint positions via IK
        std::array<double, 7> target_q = cartesianPoseToJointPosition(desired_pose, current_state_);
        franka::JointPositions joint_positions{target_q};
        joint_positions.motion_finished = desired_pose.motion_finished;

        franka::Torques torques = jointPositionToTorque(joint_positions, current_state_);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_.tau_J_d = torques.tau_J;
        }
        env_->nextStep(torques, current_state_);

        if (desired_pose.motion_finished)
        {
            running_ = false;
            break;
        }

        next_tick += control_period_;
        std::this_thread::sleep_until(next_tick);
    }
}

void MujocoRobot::control(
    std::function<franka::CartesianVelocities(const franka::RobotState&, franka::Duration)>
        motion_generator_callback,
    bool /*limit_rate*/, double /*cutoff_frequency*/)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    if (running_)
    {
        throw franka::InvalidOperationException(
            "MujocoRobot::control: A control loop is already running.");
    }

    running_ = true;
    using clock = std::chrono::steady_clock;
    auto next_tick = clock::now();
    auto last_tick = next_tick;

    while (running_)
    {
        const auto now = clock::now();
        auto dt_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_tick);
        last_tick = now;

        franka::CartesianVelocities desired_velocities =
            motion_generator_callback(current_state_, toDuration(dt_ns));

        // Convert Cartesian velocities to joint velocities
        std::array<double, 7> target_dq =
            cartesianVelocityToJointVelocity(desired_velocities, current_state_);
        franka::JointVelocities joint_velocities{target_dq};
        joint_velocities.motion_finished = desired_velocities.motion_finished;

        franka::Torques torques = jointVelocityToTorque(joint_velocities, current_state_);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_.tau_J_d = torques.tau_J;
        }
        env_->nextStep(torques, current_state_);

        if (desired_velocities.motion_finished)
        {
            running_ = false;
            break;
        }

        next_tick += control_period_;
        std::this_thread::sleep_until(next_tick);
    }
}
