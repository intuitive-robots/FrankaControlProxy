cd build
rm -rf ./*
echo ${PWD}/../../ZeroLanCom/build
cmake ..
make -j$(nproc)