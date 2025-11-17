cd build
cmake -DLOCAL_TESTING=ON ..
make -j8
# ./proxy ../local_test.yaml