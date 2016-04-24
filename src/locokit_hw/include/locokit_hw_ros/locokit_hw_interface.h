#ifndef LOCOKIT_HW_INTERFACE_H
#define LOCOKIT_HW_INTERFACE_H

//ROS CONTROL
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Duration.h>


//ROS
#include <ros/ros.h>

//STANDARD
#include <signal.h>
#include <time.h>

//LOCOKIT INTERFACE
#include <locokit_firmware/LocoKitInterface.h>
#include <locokit/robot_configuration.h>


typedef double sensor;
typedef double motor;


class LocokitHW : public hardware_interface::RobotHW
{
public:
  LocokitHW(ros::NodeHandle nh);
  bool configure();
  bool start();
  bool read();
  void write();
  virtual ~LocokitHW();

private:
  ros::NodeHandle nh_;
  bool tcp_connected_;
  LocoKitInterface* locokit_interface_;
  int sensor_number_;
  int motor_number_;
  int step_count_;
  sensor* sensors_;
  motor* motors_;
};

#endif // ifndef LOCOKIT_HW_INTERFACE_H

