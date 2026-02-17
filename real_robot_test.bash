mkdir build
cd build
cmake -DLOCAL_TESTING=OFF ..
make -j8
cd ..
./build/proxy ./config/proxy/droid.yaml