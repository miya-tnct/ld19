#!/bin/bash

# Exit on error
set -e
set -x

if grep -q Raspberry /proc/cpuinfo; then
    echo "Running on a Raspberry Pi"
else
    echo "Not running on a Raspberry Pi. Use at your own risk!"
fi

WORK_DIR=ld06
mkdir $WORK_DIR

echo "Clone the driver repo"
git clone https://github.com/LetsOKdo/ld19.git

echo "Copy driver source"
cp -r ld19/SBC_ROS_SDK/src $WORK_DIR

echo "Copy rviz source"
cp -r ld19/SBC_ROS_SDK/rviz $WORK_DIR

#echo "Changing to working director"
cd $WORK_DIR

#echo "Build driver"
catkin_make
