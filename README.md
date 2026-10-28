# Franka Control Proxy

A C++ server for controlling Franka Emika robots with optional MuJoCo simulation support.

## ✨ Features

- Real-time control interface for Franka Emika Panda robots
- Hybrid joint impedance control mode
- Gravity compensation control
- Optional MuJoCo simulation for testing without hardware
- Support for Robotiq grippers

---

## 🛠 Building

All build dependencies (compiler, CMake, pinocchio 2.5.2, libfranka 0.9.2, Boost, ...) come from the
[pixi](https://pixi.sh) environment in `pixi.toml`. ruckig and ZeroLanCom are fetched by `CMakeLists.txt`.

```bash
pixi run build    # -> build/proxy
```

libfranka must match the robot's system version; change `libfranka` in `pixi.toml` if your robot needs another one.

The MuJoCo simulation build (`-DNO_ROBOT_TESTING=ON`) additionally needs MuJoCo 3.4.0 and GLFW, which are not part
of the pixi environment.

---

## 🚀 Deploying to the real-time machine

The real-time machine (`mobile-0-rt`) has no internet, so the proxy is built elsewhere and only what it
needs at runtime is copied over:

```bash
pixi run dist     # -> dist/{bin/proxy, lib/*.so, config/, models/franka_emika_panda/panda_arm.urdf}
pixi run deploy   # replaces mobile-0-rt:~/ngres/franka_control_proxy with dist/
```

`bin/proxy` finds its libraries in `../lib`, so no environment or `LD_LIBRARY_PATH` is needed on the target.

---

## 🏃 Usage

Run from the folder that contains `config/` and `models/` (paths in the configs are relative to it):

```bash
./bin/proxy ./config/proxy/droid.yaml     # deployed folder
./build/proxy ./config/proxy/droid.yaml   # local build
```

See the `config/` directory for example configuration files.

---

## 📁 Project Structure

```
FrankaControlProxy/
├── config/              # Configuration files
│   ├── controller/      # Controller parameters
│   ├── gripper/         # Gripper configurations
│   ├── proxy/           # Proxy server settings
│   └── robot/           # Robot configurations
├── include/             # Header files
│   ├── control_mode/    # Control mode interfaces
│   ├── mujoco_sim/      # MuJoCo simulation
│   ├── protocol/        # Communication protocol
│   ├── robots/          # Robot interfaces
│   └── utils/           # Utility functions
├── models/              # MuJoCo robot models
├── scripts/             # Build and utility scripts
├── src/                 # Source files
└── third_party/         # Third-party dependencies
```

---

## 📄 License

See [LICENSE](LICENSE) for details.
