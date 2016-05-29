#!/bin/bash

# Getting current folder
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $SCRIPT_DIR
cd ../..
LEGS_FOLDER=$(pwd)

# Pulling the submodules
echo "    Pulling submodules"
cd $LEGS_FOLDER/src
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

cd realtime_tools
git checkout kinetic-devel
cd. ..

# Installing Catkin Tools
echo "    Installing Catkin Tools"
sudo apt-get install python-catkin-tools
wait

#Installing Gazebo 7
echo "    Installing Gazebo 7"
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get install gazebo7 libgazebo7-dev
wait

#Installing some extra tools
echo "    Installing some extra tools"
sudo apt-get install liburdfdom-tools

#Adding the Legs model to the simulator
echo "    Adding the Legs files to the bashrc"
echo "

### RuBi ###
export LEGS=\"$LEGS_FOLDER\"

### ROS ###
source \$LEGS/src/devel/setup.bash

### Gazebo ###
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=\${GAZEBO_MODEL_PATH}:\$LEGS/src/rubi_description:\$LEGS/src/gazebo_resources/models
export GAZEBO_RESOURCE_PATH=\${GAZEBO_RESOURCE_PATH}:\$LEGS/src/gazebo_resources" >> $HOME/.bashrc

#Add execute permissions to some files
echo "    Adding execute permissions"
cd $LEGS_FOLDER
chmod +x src/rubi_controllers/cfg/impulse.cfg
chmod +x src/rubi_controllers/cfg/two_neuron.cfg
chmod +x src/dacbot_controllers/cfg/two_neuron.cfg

echo "Done! :)"
