cmake_minimum_required(VERSION 2.8.3)
project(locokit_hw)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin COMPONENTS
	roscpp
        rospy
        std_msgs
        controller_manager
	)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES locokit_hw
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


###########
## Build ##
###########


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    $ENV{GOROBOTS}
    $ENV{GOROBOTS}/utils/real_robots/locokit
)
set(HEADERS
  include/locokit_hw_ros/locokit_hw_interface.h
  include/locokit_fimrware/LocoKitInterface.h
  include/locokit_firmware/ConnectionClass.h
  include/locokit/robot_configuration.h
  include/locokit/locokit_TCP.h
)

## Declare a C++ executable
add_executable(locokit_hw_node src/locokit_hw_interface_node.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(locokit_hw_node
   ${catkin_LIBRARIES}
 )
