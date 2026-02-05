# Franka Control Proxy

This project is a C++-based server for controlling Franka Emika robots.

---

## 🛠 Installation & Setup

Ensure your environment is configured correctly by following these steps:

1. Create Environment

```bash
conda create -n franka_control_proxy python=3.9
conda activate franka_control_proxy
conda install pinocchio==2.5.2
```

2. Install pinocchio (v2.5.2):

```bash
conda activate franka_control_proxy
conda install pinocchio==2.5.2
```

3. Install Zerolancom

- Pinocchio (v2.5.2):
```bash
cd [some_place]
git clone git@github.com:xinkai-jiang/ZeroLanCom.git
cd ZeroLanCom
bash ./scripts/compile.sh
```

4. Install libfranka
- libfranka (v0.9.2)
```bash
git clone git@github.com:frankarobotics/libfranka.git
git checkout v0.9.2
```

4. Install Mujoco (Optional)

If you don't need to test your controller in the simulation, skip this step.

Download Mujoco-3.4.0 to ~/.mujoco and unzip
```
https://github.com/google-deepmind/mujoco/releases/download/3.4.0/mujoco-3.4.0-linux-x86_64.tar.gz
```


```Bash
sudo apt install libglfw3-dev
bash ./scripts/compile_with_mujoco.sh
```
