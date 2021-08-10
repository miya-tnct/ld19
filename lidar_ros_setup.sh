#!/bin/bash

# Exit on error
set -e
set -x

if grep -q Raspberry /proc/cpuinfo; then
    echo "Running on a Raspberry Pi"
else
    echo "Not running on a Raspberry Pi. Use at your own risk!"
fi

echo "Update sources list"
sudo sed -i 's!^deb http://raspbian.raspberrypi.org/raspbian/.*$!deb http://mirrors.ustc.edu.cn/raspbian/raspbian/ buster main contrib non-free rpi!g' /etc/apt/sources.list 

echo "STEP1: Install Dependencies and Download ROS source packages"

# Where will the output go?
WORK_DIR="/home/pi/ros_catkin_ws"

BUILD_DEPS="build-essential cmake"
PYTHON_DEPS="python-rosdep python-rosinstall-generator python-wstool python-rosinstall"

echo "Adding ros repo https://www.ros.org/"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
#wget -qO - http://packages.ros.org/ros.key | sudo apt-key add -

echo "Install dependencies"
sudo apt update
sudo apt install -y $BUILD_DEPS $PYTHON_DEPS

# Note remove /etc/ros/rosdep/sources.list.d/20-default.list
# to re-run initialisation
echo "Initialising ROS"
sudo rosdep init
rosdep update

echo "STEP2(OPTIONAL if necessary): Solve the ERROR: Edit script"
#ERROR :cannot download default sources list from: https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list Website may be down.

#sudo vim /etc/hosts
#Add 151.101.76.133    raw.githubusercontent.com
#Then reinitialize rosdep and update it
#sudo rosdep init
#rosdep update


echo "STEP3: Install Melodic Desktop"
mkdir $WORK_DIR
cd $WORK_DIR

rosinstall_generator desktop --rosdistro melodic --deps --wet-only --tar > melodic-desktop-wet.rosinstall
wstool init -j8 src melodic-desktop-wet.rosinstall

echo "If wstool init fails or is interrupted, you can resume the download by running:"
#wstool update -j4 -t src

echo "STEP4: Fix the Issues"
# Install compatible version of Assimp (Open Asset Import Library) to fix collada_urdf dependency problem.
mkdir $WORK_DIR/external_src
cd $WORK_DIR/external_src
wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
unzip assimp-3.1.1_no_test_models.zip
cd assimp-3.1.1
cmake .
make
sudo make install

echo "Install OGRE for rviz"
sudo apt install -y libogre-1.9-dev

cd $WORK_DIR

echo "Install other deps using rosdep"
rosdep install --from-paths src --ignore-src --rosdistro melodic -y

echo "STEP5: Build and Source the Installation"
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j2

echo "Update bashrc"
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# *************************
# End of ROS build
# reboot and test with:
# roscore
