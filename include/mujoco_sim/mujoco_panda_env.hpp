#pragma once

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <mujoco/mujoco.h>

#include <memory>
#include <mutex>
#include <string>

#include "utils/config_file_reader.hpp"

struct MujocoEnvConfig
{
    // Control frequency (Hz)
    int control_rate{1000};

    // Viewer settings
    bool enable_viewer{true};
    int render_fps{60};

    // End-effector body name
    std::string ee_body_name{"hand"};

    MujocoEnvConfig() = default;

    void fromFile(const std::string& config_path)
    {
        ConfigFileReader reader(config_path);
        control_rate = reader.getValue<int>("control_rate", 1000);
        enable_viewer = reader.getValue<bool>("enable_viewer", true);
        render_fps = reader.getValue<int>("render_fps", 60);
        ee_body_name = reader.getValue<std::string>("ee_body_name", "hand");
    }
};

// Simple Panda environment that advances MuJoCo on demand via nextStep.
// Optionally spawns a MujocoViewer to visualize the latest state snapshot.
class MujocoPandaEnv
{
  public:
    explicit MujocoPandaEnv(const std::string& model_path,
                            const MujocoEnvConfig& config = MujocoEnvConfig{});
    ~MujocoPandaEnv();

    // Loads the model/data. Optionally launches a viewer window.
    void start();
    void stop();
    // Advance one simulation step with provided joint torques.
    void nextStep(const franka::Torques& torques, franka::RobotState& robot_state);

    void updateStateSnapshot(mjData& out_copy) const;
    void refreshRobotState(franka::RobotState& robot_state);

    mjModel* getModel() const
    {
        return model_.get();
    }
    mjData* getData() const
    {
        return data_.get();
    }
    const MujocoEnvConfig& getConfig() const
    {
        return config_;
    }

  private:
    void loadModel();
    std::string model_path_;
    MujocoEnvConfig config_;
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
    bool initialized_{false};
    int ee_body_id_{-1}; // Cached body ID for end-effector
    void _refreshRobotState(franka::RobotState& robot_state);
    mutable std::mutex data_mutex_;
};
