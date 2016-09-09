# AutonomousFlight

Indoor autonomous flight using a downward facing camera and multiple apriltags for localization, together with a ring of 8 range sensors for obstacle avoidance.

### Equipment used:

* MatrixVision Camera
* Odroid XU4
* Pixhawk Flight Controller with PX4 stack
* Teraranger Tower

![1-1](https://raw.githubusercontent.com/akshayshetty/images/master/AutonomousFlight.png)

## Setting up new SD card for Odroid:

### OS and Memory:

* From http://odroid.in/ubuntu_14.04lts/ download *ubuntu-14.04.1lts-lubuntu-odroid-xu3-20150212.img.xz* image file to your computer. Flash this image onto the SD card
* Power up Odroid with the SD card, and expand memory using Odroid Utility and reboot

### ROS barebones:

* Download the [ros-setup.sh](https://github.com/abencomo/AutonomousFlight/blob/master/setup/ros-setup.sh) script from the setup folder
* In the download directory, execute *chmod a+x ./ros-setup.sh* to make the script file executable
* Execute *./ros-setup.sh* to install ROS Indigo barebones. This may take a few minutes
 
### AutonomousFlight workspace:

* Download the [AutonomousFlight.sh](https://github.com/abencomo/AutonomousFlight/blob/master/setup/AutonomousFlight.sh) script from the setup folder
* In the download directory, execute *chmod a+x ./AutonomousFlight.sh* to make the script file executable
* Execute *./AutonomousFlight.sh* to install all the required packages. This may take a few minutes
