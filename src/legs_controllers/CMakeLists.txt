cmake_minimum_required(VERSION 2.8.3)
project(legs_controllers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    std_srvs
    message_generation
    sensor_msgs
    gazebo_msgs
    dynamic_reconfigure
    roslib
    )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   impulse_one_leg.srv
   impulse_two_legs.srv
   impulse_from_file.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  std_srvs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/two_neuron.cfg
    cfg/impulse.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES legs_controllers
  CATKIN_DEPENDS std_msgs sensor_msgs gazebo_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
    ${catkin_INCLUDE_DIRS}
    $ENV{GOROBOTS}
    $ENV{GOROBOTS}/controllers/runbotii_dacbot
    $ENV{GOROBOTS}/utils/ann-framework
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -Wextra")
set(CMAKE_BUILD_TYPE Debug)

### Muscle controller
add_executable(muscle_controller_node
    src/muscle_runbot_controller.cpp
    $ENV{GOROBOTS}/controllers/runbotii_dacbot/cnnet.cpp
    $ENV{GOROBOTS}/controllers/runbotii_dacbot/ctextarraywriter.cpp
    $ENV{GOROBOTS}/controllers/runbotii_dacbot/derivativeTransitionRegister.cpp
    $ENV{GOROBOTS}/controllers/runbotii_dacbot/plastic.cpp
    $ENV{GOROBOTS}/controllers/runbotii_dacbot/lowPassfilter.cpp
    $ENV{GOROBOTS}/controllers/runbotii_dacbot/shiftregister.cpp
    $ENV{GOROBOTS}/controllers/runbotii_dacbot/DynamicCpg.cpp
    $ENV{GOROBOTS}/controllers/runbotii_dacbot/DynamicVector.cpp
)

target_link_libraries(muscle_controller_node
    ${catkin_LIBRARIES}
)

### Two Neuron Controller
file(GLOB ann_framework_folder
    $ENV{GOROBOTS}/utils/ann-framework/*.cpp
    )
add_executable(two_neuron_controller_node
    src/two_neuron_controller.cpp
    ${ann_framework_folder}
    )
target_link_libraries(two_neuron_controller_node
    ${catkin_LIBRARIES}
    )
add_dependencies(two_neuron_controller_node
    ${PROJECT_NAME}_gencfg)

### Impulse controller
add_executable(impulse_controller_node
    src/impulse_controller.cpp
    )
target_link_libraries(impulse_controller_node
    ${catkin_LIBRARIES} yaml-cpp
    )
add_dependencies(impulse_controller_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg
    )
