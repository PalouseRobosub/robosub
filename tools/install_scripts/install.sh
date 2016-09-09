#!/bin/bash

#setup gitconfig
. git_setup.sh

# towards the end of this script, we will execute
# 'sudo aptitude install $PACKAGES', so any new
# packages should get appended to this variable,
# instead of directly installing
PACKAGES="
"


#setup ROS stuff
sudo sh -c '. /etc/lsb-release && echo "deb http://mirror.umd.edu/packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
PACKAGES="$PACKAGES ros-indigo-ros-base"

PACKAGES="$PACKAGES
git
build-essential
cmake
libeigen3-dev
libzmq3-dev
python-zmq 
p7zip-full
wget
libraw1394-11
libgtkmm-2.4-1c2a
libglademm-2.4-1c2a
libgtkglextmm-x11-1.2-dev
libgtkglextmm-x11-1.2
libusb-1.0-0
"
# update
sudo aptitude update

echo "Installing: $PACKAGES"
sudo aptitude install $PACKAGES

#
#grab stuff from from the robosub server
#
pushd ~/Downloads
#opencv3
wget http://robosub.eecs.wsu.edu/build_outputs/opencv_3.1.0.deb
sudo dpkg -i opencv_3.1.0.deb
#rapidjson
wget http://robosub.eecs.wsu.edu/build_outputs/rapidjson_1.0.4.deb
sudo dpkg -i rapidjson_1.0.4.deb
#flycap software (for cameras)
wget http://robosub.eecs.wsu.edu/build_outputs/flycapture.7z
7z x flycapture.7z
cd flycapture
sudo bash install_flycapture.sh
#tig (newer version, the one in the repositories is very old)
wget http://robosub.eecs.wsu.edu/build_outputs/tig_2.2-1.deb
sudo dpkg -i tig_2.2-1.deb
popd

# Google test
#sudo apt-get install libgtest-dev
#sudo cd /usr/src/gtest
#sudo cmake CMakeLists.txt
#sudo make
#sudo mv *.a /usr/lib

