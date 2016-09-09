#! /bin/bash

sudo apt-get update

#setup editor
sudo apt-get install -y gedit
sudo apt-get purge -y leafpad

#initial settings for ROS
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
#sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update

#install ROS
sudo apt-get install -y ros-indigo-ros-base
sudo apt-get install -y python-rosdep
sudo rosdep init
rosdep update
sudo apt-get install -y python-rosinstall

#add ROS environment variables to bash
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc


