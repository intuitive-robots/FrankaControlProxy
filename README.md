# Franka Control Proxy

A C++ server for controlling Franka Emika robots with optional MuJoCo simulation support.

## ✨ Features

- Real-time control interface for Franka Emika Panda robots
- Hybrid joint impedance control mode
- Gravity compensation control
- Optional MuJoCo simulation for testing without hardware
- Support for Robotiq grippers

---

## 📋 Prerequisites

- **OS**: Linux (tested on Ubuntu 20.04/22.04)
- **Compiler**: GCC with C++17 support
- **CMake**: ≥ 3.16
- **Conda**: For environment management

---

## 🛠 Installation

### 1. Create Conda Environment

```bash
conda create -n franka_control_proxy python=3.9
conda activate franka_control_proxy
```

### 2. Install Pinocchio

```bash
conda install pinocchio==2.5.2
```

### 3. Install ZeroLanCom

```bash
cd <your_workspace>
git clone git@github.com:xinkai-jiang/ZeroLanCom.git
cd ZeroLanCom
bash ./scripts/compile.sh
```

### 4. Install libfranka

```bash
cd <your_workspace>
git clone git@github.com:frankaemika/libfranka.git
cd libfranka
git checkout v0.9.2
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

### 5. (Optional) Install MuJoCo for Simulation

Skip this step if you only need to control real robots.

1. Download and extract MuJoCo 3.4.0:

```bash
mkdir -p ~/.mujoco
cd ~/.mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/3.4.0/mujoco-3.4.0-linux-x86_64.tar.gz
tar -xzf mujoco-3.4.0-linux-x86_64.tar.gz
```

2. Install GLFW:

```bash
sudo apt install libglfw3-dev
```

---

## 🔨 Building

### Build for Real Robot

```bash
bash ./scripts/compile.sh
```

### Build with MuJoCo Simulation

```bash
bash ./scripts/compile_with_mujoco.sh
```

---

## 🚀 Usage

```bash
./build/proxy --config <config_file>
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
