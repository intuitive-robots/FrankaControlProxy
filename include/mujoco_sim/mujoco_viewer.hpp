#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "mujoco_sim/mujoco_panda_env.hpp"

// MuJoCo viewer that renders externally simulated state on its own thread
// (~60 Hz by default).
class MujocoViewer {
 public:
  explicit MujocoViewer(MujocoPandaEnv* env);
  ~MujocoViewer();

  // Starts the rendering thread and opens a window. Returns false on failure.
  void start(int width = 1280, int height = 720, const char* title = "MuJoCo");
  void stop();

  // Adjust desired render rate (defaults to ~60 Hz).
  void setRenderHz(double hz);

  // Thread-safe: copy an externally simulated mjData into the viewer buffer.
  void updateState(const mjData& state);

 private:
  struct MouseState {
    bool left_down = false;
    bool right_down = false;
    double last_x = 0.0;
    double last_y = 0.0;
  };

  void renderLoop();
  bool initGlfw(int width, int height, const char* title);
  void shutdownGlfw();

  static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
  static void CursorPosCallback(GLFWwindow* window, double xpos, double ypos);
  static void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);

  void handleMouseButton(int button, int action);
  void handleCursorPos(double xpos, double ypos);
  void handleScroll(double yoffset);

  MujocoPandaEnv* env_;
  mjModel* model_;
  std::unique_ptr<mjData> snapshot_{nullptr};

  std::atomic<bool> running_{false};
  std::thread render_thread_;

  GLFWwindow* window_{nullptr};
  mjvCamera cam_{};
  mjvOption opt_{};
  mjvScene scn_{};
  mjrContext con_{};

  MouseState mouse_state_;
  double render_dt_{1.0 / 60.0};
  const float renderHz = 60.0;
};
