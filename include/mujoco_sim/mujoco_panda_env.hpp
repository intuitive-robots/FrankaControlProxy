#pragma once

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <mujoco/mujoco.h>

#include <memory>
#include <mutex>
#include <string>

// Simple Panda environment that advances MuJoCo on demand via nextStep.
// Optionally spawns a MujocoViewer to visualize the latest state snapshot.
class MujocoPandaEnv
{
  public:
    explicit MujocoPandaEnv(const std::string& model_path);
    ~MujocoPandaEnv();

    // Loads the model/data. Optionally launches a viewer window.
    bool start();
    void stop();
    // Advance one simulation step with provided joint torques.
    void nextStep(const franka::Torques& torques, franka::RobotState& robot_state);

    bool getStateSnapshot(mjData& out_copy) const;
    void refreshRobotState(franka::RobotState& robot_state);
    mjModel* getModel() const
    {
        return model_.get();
    }
    mjData* getData() const
    {
        return data_.get();
    }

  private:
    bool loadModel();
    std::string model_path_;
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
    bool initialized_{false};

    mutable std::mutex state_mutex_;
};
