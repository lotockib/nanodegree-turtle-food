#!/bin/bash

# Setup bash rc file
bashrc="$HOME/.bashrc"
while IFS= read line
do
    if [[ $line == *"source /opt/ros/"* ]]; then

        if [[ $line != "source /opt/ros/kinetic/setup.bash" ]]; then
            echo "ERROR: invalid source command found in ~/.bashrc: $line"
            echo "Remove this line from your .bashrc file and try again."
            echo "Only 'source /opt/ros/kinetic/setup.bash' is allowed'."
            exit 1
        fi
    fi
done <"$bashrc"
echo "export PATH="~/miniconda3/bin:$PATH"" >> ~/.bashrc
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

# Setup sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update

# Install dependencies
sudo apt install python-catkin-pkg python-rospkg -y
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential ros-kinetic-rqt ros-kinetic-rqt-common-plugins -y
pip install numpy pyyaml pyside2 pyqtgraph matplotlib pydot rospkg
sudo apt install --reinstall libxcb-xinerama0 -y # workaround for issue on vm machine
sudo apt-get install libcanberra-gtk-module:i386 -y # fix for issue on my vm machine

# Install ros
sudo apt install ros-kinetic-desktop-full -y

# setup ros
sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init --reinstall
rosdep update --include-eol-distros

# build workspace
source /ros/opt/kinetic/setup.bash
cd catkin_ws
catkin_make