#!/bin/bash

# Getting current folder
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $SCRIPT_DIR
cd ..
LEGS_FOLDER=$(pwd)

# Pulling the submodules
echo "    Pulling submodules"
cd $LEGS_FOLDER
git submodule update --init --recursive

cd control_toolbox
git checkout 1.13.2
cd ..

cd gazebo_ros_pkgs
git checkout jade-devel
cd ..

cd ros_control
git checkout jade-devel
cd ..

cd ros_controllers
git checkout jade-devel
cd ..

# Installing Catkin Tools
echo "    Installing Catkin Tools"
sudo apt-get install python-catkin-tools
wait

#Installing Gazebo 6
echo "    Installing Gazebo 7"
# Add the OSRF repository
if [ ! -e /etc/apt/sources.list.d/gazebo-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.osrfoundation.org/gazebo/ubuntu ${codename} main\" > /etc/apt/sources.list.d/gazebo-latest.list"
fi

# Download the OSRF keys
has_key=`apt-key list | grep "OSRF deb-builder"`

if [ -z "$has_key" ]; then
  wget --quiet http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
fi
sudo apt-get install gazebo7 libgazebo7-dev
sudo apt-get install python-catkin-tools
wait

#Installing some extra tools
echo "    Installing some extra tools"
sudo apt-get install liburdfdom-tools

#Downloading Gazebo Models from the repo
echo "    Downloading Gazebo Models from the repo"
cd $HOME/.gazebo
mv models models_orig
hg clone https://bitbucket.org/osrf/gazebo_models
wait
mv gazebo_models models

#Adding the Legs model to the simulator
echo "    Adding the Legs files to the bashrc"
echo "

### Owns ###
export LEGS=\"$LEGS_FOLDER\"

### ROS ###
source \$LEGS/devel/setup.bash

### Gazebo ###
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=\${GAZEBO_MODEL_PATH}:\$LEGS/src:\$LEGS/src/gazebo/models
export GAZEBO_RESOURCE_PATH=\${GAZEBO_RESOURCE_PATH}:\$LEGS/src/gazebo" >> $HOME/.bashrc

echo "Done! :)"
