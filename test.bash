cd build
cmake -DCMAKE_PREFIX_PATH=/home/alr_admin/robot/libfranka/build/ ..
make
./robot_config /home/alr_admin/robot/franka_zmq/config.cfg