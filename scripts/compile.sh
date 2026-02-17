cd build
rm -rf ./*
echo "Compiling ZeroLanCom..."
echo ${PWD}/../../ZeroLanCom/build
cmake .. -DCMAKE_PREFIX_PATH=${PWD}/../../ZeroLanCom/build
make -j$(nproc)