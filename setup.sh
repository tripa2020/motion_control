#!/bin/bash

## HW5 Setup Script.
## Places the HW5 code into the ~/homework_ws and builds the packages.

## Preconditions:
## - The hw5_control repo was downloaded as a Zip file from GitHub to ~/Downloads
## - If the zip file is named differently in your Downloads folder, modify the commands accordingly.
## - The ~/homework_ws/src folder has been created, as per the pre-HW instructions.

cd ~/Downloads
unzip hw5_control-main.zip
mv ~/Downloads/hw5_control-main ~/homework_ws/src
mv ~/homework_ws/src/hw5_control-main ~/homework_ws/src/hw5_control

# build and source dependencies_ws and interbotix_ws

cd ~/homework_ws
catkin clean
catkin build
source ~/homework_ws/devel/setup.bash
