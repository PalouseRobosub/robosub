#!/bin/bash

sudo sh -c 'echo "deb http://robosub.eecs.wsu.edu/repo/ /" > /etc/apt/sources.list.d/robosub.list'
wget http://robosub.eecs.wsu.edu/repo/repository_key -O - | sudo apt-key add -

sudo sh -c 'echo "deb http://mirror.umd.edu/packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -


sudo aptitude update
sudo aptitude install robosub
