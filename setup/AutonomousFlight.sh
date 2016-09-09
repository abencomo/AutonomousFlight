#! /bin/bash

mkdir ~/Documents/AutonomousFlight
mkdir ~/Documents/AutonomousFlight/src
cd ~/Documents/AutonomousFlight/src
source ~/.bashrc
catkin_init_workspace

sudo apt-get install -y ros-indigo-camera-info-manager
sudo apt-get install -y ros-indigo-diagnostic-updater
sudo apt-get install -y ros-indigo-mavros*
sudo apt-get install -y libcgal-dev
sudo apt-get install -y xterm

git clone https://github.com/KumarRobotics/camera_base.git
git clone https://github.com/KumarRobotics/bluefox2.git
git clone https://github.com/personalrobotics/apriltags.git
git clone https://github.com/Terabee/terarangertower-ros.git

cd
git clone https://github.com/abencomo/AutonomousFlight.git

sudo mv ~/AutonomousFlight/planner ~/Documents/AutonomousFlight/src/
sudo mv ~/AutonomousFlight/estimator ~/Documents/AutonomousFlight/src/

sudo mv ~/AutonomousFlight/camera_apriltags_tower_mavros.launch ~/Documents/AutonomousFlight/
sudo mv ~/AutonomousFlight/AutonomousFlight.launch ~/Documents/AutonomousFlight/

sudo mkdir ~/.ros/camera_info
sudo mv ~/AutonomousFlight/mv_25001964.yaml ~/.ros/camera_info/

cd ~/Documents/AutonomousFlight/
source ~/.bashrc
catkin_make -j1

sudo mv ~/AutonomousFlight/change_navigation_mode.sh ~/
sudo chmod a+x ~/change_navigation_mode.sh

sudo mv ~/AutonomousFlight/indoor-go.sh ~/
sudo chmod a+x ~/indoor-go.sh

sudo rm -rf ~/AutonomousFlight
