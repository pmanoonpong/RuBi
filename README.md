<h1>Design, simulation and development of RuBi, a bipedal platform for human-like locomotion studies</h1>

<h2>Description</h2>
RuBi is a study framework for human-like gaits part of the master thesis of Ignacio Torroba and Jorge Rodriguez. The idea is to create a robot that can perform actions like walking, running or jumping both in simulation in real life. One of the main advantages is the possibility of changing the springs in order to study SEA, PEA or SEA+PEA configurations with different springs.

![alt tag](https://github.com/RuBi-Robot/RuBi/blob/master/media/Renders/Legs-Camera%201.25.jpg)

In here, the description of the structure of the project is presented. The how-to for starting the simulation or bring up the robot can also be found in here. Some example controllers are given too.

It is supposed that you have a ROS Jade installed and working. This has only been tested in Ubuntu 15.04 and 14.04 under ROS Jade. To install it you can follow the official guides to be found in: http://wiki.ros.org/jade/Installation/Ubuntu. Remember that ROS Jade ONLY supports Trusty (14.04), Utopic (14.10) and Vivid (15.04) for Debian packages.

<h2>Project structure</h2>
The project has three basic folders:
  * <b>docs</b>: in here all the documents used for the deployment of the project are found. The master thesis is under the folder "report".
  * <b>media</b>: where all the media files as Photos, Videos, Presentations are stored.
  * <b>src</b>: the source folder where all the code, CAD models, robots description are.

The important folder for the user is the <b>src</b> folder and it follows the ROS structure convention. The code style is the imposed by the <i>Google Style</i> defined by the clang-code-model. The <b>build system</b> is catkin tools, and the environment is already included in the repo. This means that the user only need to clone the repo and everything will work "out-of-the-box" (notice the quotes there please).

In the source folder three kind of packages can be found:
  *   <b>ROS Jade needs</b>: Due to the use of ROS Jade, needed due to that in the description of the robots we use Jade-exclusive functions, some of the packages need to be compiled from the source. Those packages are copied as submodules in the repo and the correct branches are set with the installation script.
  *   <b>Robot descriptions</b>: Based on the Gazebo-ROS convention for describe the robots (http://gazebosim.org/tutorials/?tut=ros_overview). It is important to notice that TWO robots have been implemented: <b>RuBi</b> and <b>Dacbot</b>. Dacbot was created as a practice to finally create RuBi. Then the five folders are found for each robot.
    - robot_descrition: where all the files that describe the robot are found. If any CAD model is developed must be added in here.
    - robot_gazebo: used as a wrapper for bringing up the robot's simulation.
    - robot_bringup: used as a wrapper for bringing up the real robot.
    - robot_joint_controllers: originally called robot_controllers, though the name was change due to the lexical problem when defining "controller". This package offers the <i>joint controller</i> inside of ROS control. Ultimately what creates is a topic (e.g. /rubi/left_ankle_command) that can move the joint. This topic is then interfaced with the simulation or the real robot through a <i>hardware_interface</i>. Gazebo offers one, and the one from the real robot is called <b>locokit_hw</b> and can be found as another package within <i>src</i>.
    -   robot_controllers: Additionally, we add a new folder in which we deploy the controllers of the robot. Meaning, the controller that actually moves the robot for walking, running or jumping.
    
  *   <b>Miscellaneous</b>: these are <i>scripts</i> and <i>matlab_scripts</i>. The <b>scripts</b> folder includes the installation folder that, once the repo is cloned will download the submodules, install dependencies, etc... the <b>matlab_scripts</b> in where the actual Matlab programs used to size the motors, calculate the kinematic and dynamic models of the robot, etc.. are to be found.

<h2>First time</h2>
As explained before, inside of "src/scripts/" a script to be run the first time is found. This will download the submodules, install dependencies, etc... So:
``` bash
git clone URL_of_the_project
cd (path_to_project)/src/scripts
./install_first_time.sh
```
<h2>Robot definition</h2>
Inside the <i>rubi_definition</i> folder, several folders can be found. The suggested structured given in the ROS-Gazebo guides is followed. Two important things:
  * <b>CAD model</b>: the CAD model of the robot is to be found in the folder <b>meshes</b>. In here, the SolidWorks files are stored. At the same time, these follow the order of having the assembly file in the root folder and the parts in sub-folders. This will leave an assembly file called <i>Legs</i> that will load the Legs model and another one called <i>Legs with environment</i> where the robot is loaded along with the treadmill.
  * <b>Xacro</b>: the robot is defined in the URDF format. This is needed because ROS Control uses it. The robot is coded using Xacro because it simplifies the wrtting and allows conditional expression. Check the flags out inside of the <i>rubi.xacro</i> file. These add some functionality like scaling the robot or create the rotational holder, among others.

<h2>Develop controllers</h2>
In order to facilitate the development of new controllers two example controllers are given inside of RuBi. First clarify that ALL the controllers of a robot MUST be in one folder. This was decided due to the experience obtained during the use of LPZ Robots. 

To deploy a new controller just add the source into the <b>src</b> folder and add it into the CMakeList.txt. Is a ROS package, so more information can be found in Google. One of the example controllers show how to use GoRobots in it.

Remember that for using GoRobots an environmental variable called <i>GOROBOTS</i> must exist. This can be done by adding to the <i>.bashrc</i> file found in the home folder:
``` bash
export GOROBOTS=/path/to/gorobots
```

All the packages in the catkin environment are compiled introducing the next comand in any of the sub-directories of the project:
``` bash
catkin build
```

<h2>Bring up the simulation</h2>
The simulation can be loaded by opening a terminal and writing:
``` bash
roslaunch rubi_gazebo rubi.world
```
This will load the Gazebo environment, the model, the joint controllers and the plugins associated. Once finished, the controllers can be started.

<h2>Bring up the real robot</h2>
The real robot can be loaded by opening a terminal and writing:
``` bash
roslaunch rubi_bringup rubi_bringup.launch
```
This will load the Locokit interface, the model, the joint controllers and the plugins associated. Once finished, the controllers can be started.

<h2>Contributors</h2>
<b>Supervisors: </b>
  * Poramate Manoonpong: poma@mmmi.sdu.dk
  * Jørgen Christian Larsen: jcla@mmmi.sdu.dk
<b>Students: </b>
  * Jorge Rodriguez Marin: jorge.rguez.marin@gmail.com
  * Ignacio Torroba Balmori: ignaciotorroba@hotmail.com