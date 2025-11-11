cd build
cmake -DLOCAL_TESTING=ON ..
make
./proxy ../local_test.yaml