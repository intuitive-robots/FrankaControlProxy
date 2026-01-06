#include "mujoco_sim/mujoco_viewer.hpp"

#include <chrono>
#include <iostream>
#include <utility>

namespace {
constexpr int kMaxGeom = 1000;
}

MujocoViewer::MujocoViewer(MujocoPandaEnv* env)
    : env_(env) {}

MujocoViewer::~MujocoViewer() {
  stop();
}

void MujocoViewer::start(int width, int height, const char* title) {
  if (running_) {
    throw std::runtime_error("MujocoViewer is already running");
  }

  running_ = true;
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  cam_.lookat[0] = 0.0f;
  cam_.lookat[1] = 0.0f;
  cam_.lookat[2] = 0.5f;
  cam_.distance = 1.6f;
  cam_.elevation = -20.0f;
  cam_.azimuth = 90.0f;

  initGlfw(width, height, title);
  render_thread_ = std::thread(&MujocoViewer::renderLoop, this);
}

void MujocoViewer::stop() {
  running_ = false;
  if (window_) {
    glfwSetWindowShouldClose(window_, GLFW_TRUE);
    glfwPostEmptyEvent();
  }
  if (render_thread_.joinable()) {
    render_thread_.join();
  }
}

void MujocoViewer::setRenderHz(double hz) {
  if (hz > 0.0) {
    render_dt_ = 1.0 / hz;
  }
}

void MujocoViewer::renderLoop() {

  auto next = std::chrono::steady_clock::now();

  while (running_) {
    if (glfwWindowShouldClose(window_)) {
      running_ = false;
      break;
    }


    mjrRect rect{0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &rect.width, &rect.height);
    env_->getStateSnapshot(*snapshot_);
    mjv_updateScene(model_, snapshot_.get(), &opt_, nullptr, &cam_, mjCAT_ALL,
                    &scn_);
    mjr_render(rect, &scn_, &con_);
    glfwSwapBuffers(window_);
    glfwPollEvents();

    next += std::chrono::microseconds(
        static_cast<int>(render_dt_ * 1'000'000));
    std::this_thread::sleep_until(next);
  }

  shutdownGlfw();
}

bool MujocoViewer::initGlfw(int width, int height, const char* title) {
  if (!glfwInit()) {
    std::cerr << "Failed to init GLFW" << std::endl;
    return false;
  }

  window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
  if (!window_) {
    std::cerr << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return false;
  }

  glfwSetWindowUserPointer(window_, this);
  glfwSetMouseButtonCallback(window_, MouseButtonCallback);
  glfwSetCursorPosCallback(window_, CursorPosCallback);
  glfwSetScrollCallback(window_, ScrollCallback);

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  mjv_defaultScene(&scn_);
  mjr_defaultContext(&con_);
  mjv_makeScene(model_, &scn_, kMaxGeom);
  mjr_makeContext(model_, &con_, mjFONTSCALE_150);

  return true;
}

void MujocoViewer::shutdownGlfw() {
  mjr_freeContext(&con_);
  mjv_freeScene(&scn_);
  if (window_) {
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }
  glfwTerminate();
}

void MujocoViewer::MouseButtonCallback(GLFWwindow* window, int button, int action,
                                       int mods) {
  (void)mods;
  auto* self = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
  if (self) {
    self->handleMouseButton(button, action);
  }
}

void MujocoViewer::CursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
  auto* self = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
  if (self) {
    self->handleCursorPos(xpos, ypos);
  }
}

void MujocoViewer::ScrollCallback(GLFWwindow* window, double xoffset,
                                  double yoffset) {
  (void)xoffset;
  auto* self = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
  if (self) {
    self->handleScroll(yoffset);
  }
}

void MujocoViewer::handleMouseButton(int button, int action) {
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    mouse_state_.left_down = (action == GLFW_PRESS || action == GLFW_REPEAT);
  } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
    mouse_state_.right_down = (action == GLFW_PRESS || action == GLFW_REPEAT);
  }

  if (action == GLFW_PRESS && window_) {
    glfwGetCursorPos(window_, &mouse_state_.last_x, &mouse_state_.last_y);
  }
}

void MujocoViewer::handleCursorPos(double xpos, double ypos) {
  const double dx = xpos - mouse_state_.last_x;
  const double dy = ypos - mouse_state_.last_y;
  mouse_state_.last_x = xpos;
  mouse_state_.last_y = ypos;

  constexpr double scale = 0.005;
  if (mouse_state_.left_down) {
    mjv_moveCamera(model_, mjMOUSE_ROTATE_H, scale * dx, scale * dy, &scn_,
                   &cam_);
  } else if (mouse_state_.right_down) {
    mjv_moveCamera(model_, mjMOUSE_MOVE_H, scale * dx, scale * dy, &scn_,
                   &cam_);
  }
}

void MujocoViewer::handleScroll(double yoffset) {
  mjv_moveCamera(model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn_, &cam_);
}
