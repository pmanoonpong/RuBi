<h1>Design, simulation and development of a bipedal locomotion platform for human-like gaits studies</h1>

<h2>Description</h2>
This is the repository in which RuBi has been develop. RuBi is a study framework for human-like gaits part of the master thesis of Ignacio Torroba and Jorge Rodriguez. The idea is to create a robot that can perform actions like walking, r
unning or jumping both in simulation in real life. One of the main advantages is the possibility of change the springs in order to study SEA, PEA or SEA+PEA configurations.

In here, the description of the structure of the project is described. The how-to for starting the simulation or bring up the robot can also be found in here. Some example controllers are given too.

<h2>Project structure</h2>
The project has three basic folders:
  * <b>docs</b>: in here all the documents used for the deployment of the project are found. The master thesis is under the folder "report".
  * <b>media</b>: where all the media files as Photos, Videos, Presentations are stored.
  * <b>src</b>: the source folder where all the code, CAD models, robots description are.

The important folder for the user is the <b>src</b> folder and in here the ROS structure has been followed. The <b>build system</b> is catkin tools, and the environment is already included in the repo. This means that the user only need to clone the repo and everything will work "out-of-the-box" (notice the quotes there please).

In the source folder found kind of packages can be found:
  *   <b>ROS Jade needs</b>: Due to the use of ROS Jade, needed due to in the description of the robots we use jade-exclusive functions, some of the packages need to be compiled from the source. Those packages are copied as submodules in the repo and the correct branches are set with the installation script.
  *   <b>Robot descriptions</b>: Based on the Gazebo-ROS convenction for describe the robots (http://gazebosim.org/tutorials/?tut=ros_overview), four folders must be created. These are:
    - robot_descrition: where all the files that decribe the robot are found. If any CAD model is developed must be added in here.
    - robot_gazebo: used as a wrapper for bringing up the robot's simulation.
    - robot_bringup: used as a wrapper for bringing up the real robot.
    - robot_joint_controllers: originally called robot_controllers, though the name was change due to the lexical problem when defining "controller". This package offers the <i>joint controller</i> inside of ROS control. Ultimately what creates is a topic (e.g. /rubi/left_ankle_command) that can move the joint. This topic is then interfaced with the simulation or the real robot through a <i>hardware_interface</i>. Gazebo offers one, and the one from the real robot is called <b>locokit_hw</b>.
    -   robot_controllers: Additionaly, WE add a new folder in which we deploy the controllers of the robot. Meaning, the controller that actually moves the robot for walking, running or jumping.
    It is important to notice that TWO robots have been implemented: <b>RuBi</b> and <b>Dacbot</b>. Dacbot was created as a practice to finally create RuBi. Then the four folders are found for each robot.
  *   <b>Miscellaneous</b>: there are <i>scripts</i> and <i>matlab_scripts</i>. The <b>scripts</b> folder includes the installation folder that, once cloned will download the submodules, install dependencies, etc... the <b>matlab_scripts</b> in where the actual matlab programs used to size the motors, calculate the kinematic and dynamic models of the robot, etc.. are to be found.

<h2>First time</h2>
As explained before, inside of "src/scripts/" an script to be run the first time is found. This will download the submodules, install dependencies, etc... So once the project is cloned:
``` bash
git clone URL_of_the_project
cd (path_to_project)/src/scripts
./install_first_time.sh

```
<h2>Develop controllers</h2>

<h2>Contributors</h2>
<b>Supervisors: </b>

<b>Students: </b>
  * Jorge Rodriguez Marin: jorge.rguez.marin@gmail.com
  * Ignacio Torroba Balmori: ignaciotorroba@hotmail.com