#!/usr/bin/env bash

# Build
cd build || mkdir build && cd build
cmake ..
make -j4
cd ..

# Run components
cd bin || exit
./simviz & # simulation
sleep 2
./controller & # control

sleep $1 # sleep for the given period before killing

pkill -9 -P $$

echo "Done"
