#!/bin/bash

cd $HOME

xterm -hold -e "cd Documents/AutonomousFlight; source devel/setup.bash; roslaunch camera_apriltags_tower_mavros.launch" &
xterm -hold -e "sleep 5; cd Documents/AutonomousFlight; source devel/setup.bash; roslaunch AutonomousFlight.launch" &
xterm -hold -e "sleep 6; rostopic echo /scan" &
#xterm -hold -e "sleep 7; rosbag record -O latest_flight.bag /mv_25001964/image_raw" &
xterm -hold -e "sleep 8; rostopic echo /mavros/vision_pose/pose"
