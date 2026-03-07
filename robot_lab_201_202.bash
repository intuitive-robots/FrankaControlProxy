source ~/miniconda3/etc/profile.d/conda.sh
conda activate franka_control_proxy
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
./build/proxy ./config/proxy/robot_lab_teleop_201_202.yaml