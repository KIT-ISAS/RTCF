#!/bin/bash

cd "$1"
mkdir src

# get source
echo Getting sources...
cd src
git clone --recursive https://github.com/orocos-toolchain/orocos_toolchain.git
git clone https://github.com/orocos/rtt_ros_integration.git 
cd ..

# check and install dependencies
echo Getting external dependencies...
sudo apt-get install python-catkin-tools
sudo apt install ruby-backports
rosdep install --from-paths src --ignore-src -r -y

# build
echo Building...
export OROCOS_TARGET=gnulinux
catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build 

# hint for sourcing
echo Now add the following line to your .bashrc: 
echo source $(pwd)/install/setup.bash

