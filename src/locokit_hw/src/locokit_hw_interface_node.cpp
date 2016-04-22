#include "locokit_hw_ros/locokit_hw_interface.h"

bool g_quit = false;

void quitRequested(int sig) {
  g_quit = true;
}



int main( int argc, char** argv ){

  // Initialize ROS
  ros::init(argc, argv, "locokit_hw_interface");

  // Add custom signal handlers
  signal(SIGTERM, quitRequested);

  // Construct the wam structure
  ros::NodeHandle locokit_nh("locokit");


  return 0;
}
