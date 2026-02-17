mkdir build
cd build
cmake -DLOCAL_TESTING=ON ..
make -j8
cd ..
./build/proxy ./config/ProxyConfig.yaml