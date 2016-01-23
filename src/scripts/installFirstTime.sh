#!/bin/bash

#Installing Gazebo 6
echo "    Installing Gazebo 6"
sudo apt-get install gazebo6
wait

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
LEGS_SRC_FOLDER=$(pwd)
echo "

### Gazebo ###
source /usr/share/gazebo/setup.sh
export LEGS_SRC=\"$LEGS_SRC_FOLDER\"
export GAZEBO_MODEL_PATH=\${GAZEBO_MODEL_PATH}:\$LEGS_SRC/gazebo/models
export GAZEBO_RESOURCE_PATH=\${GAZEBO_RESOURCE_PATH}:\$LEGS_SRC/gazebo" >> $HOME/.bashrc

echo "Done! :)"