cd build
# rm -rf ./*
echo "Compiling ZeroLanCom..."
echo ${PWD}/../../ZeroLanCom/build
cmake .. -DNO_ROBOT_TESTING=ON
make -j$(nproc)