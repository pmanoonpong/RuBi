#!/bin/bash

cd $HOME
mkdir -p gazebo_ws
cd gazebo_ws
mkdir -p src


# Download the packages
git clone https://github.com/bulletphysics/bullet3.git
git clone https://github.com/dartsim/dart.git
hg clone https://bitbucket.org/osrf/sdformat 
hg clone https://bitbucket.org/osrf/gazebo

# Select our branches
cd gazebo
hg checkout gazebo7_7.1.0

# Add the packages.xml for catkin build
cd ..
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_bullet.xml    > bullet3/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_dart-core.xml > dart/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_gazebo.xml    > gazebo/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_sdformat.xml  > sdformat/package.xml

# Compile Gazebo with Dart and Bullet
cd ..
catkin init
catkin build -vi --cmake-args \
  -DBUILD_CORE_ONLY=ON \
  -DBUILD_SHARED_LIBS=ON \
  -DUSE_DOUBLE_PRECISION=ON

# Add the workspace to the bashrc
echo "### Gazebo Workspace ###
source $HOME/gazebo_ws/devel/setup.bash
" >> ~/.bashrc