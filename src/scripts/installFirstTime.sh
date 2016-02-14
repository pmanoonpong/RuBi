#!/bin/bash

#Installing Catkin Tools
echo "    Installing Catkin Tools"
sudo apt-get install python-catkin-tools
wait

#Installing Gazebo 6
echo "    Installing Gazebo 6"
sudo apt-get install gazebo6
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
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $SCRIPT_DIR
cd ..
LEGS_FOLDER=$(pwd)
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
