# keena
LEGO assembly robot and control on the SAI2 environment. Project for Stanford CS225A employing the Panda robotic arm.

## Requirements
You need to install and build the SAI2 environment linked [here.](https://github.com/manips-sai-org)

## Contents
* `control` contains C++ source for the controller `controller.cpp`.
* `simulation` contains C++ source for the simulation `simviz.cpp`.
* `model` contains the `urdf` files and meshes for the world and the Panda robot.

## How to build and run
Build and run with `sh buildrun.sh 10` which builds, runs, and stops the simulation automatically after 10 seconds (you can change the duration by changing the number 10).

Or
```commandline
# build
mkdir build
cd build
cmake ..
make -j4

# run (make sure redis server is running)
cd ../bin
./simviz &
./controller
```