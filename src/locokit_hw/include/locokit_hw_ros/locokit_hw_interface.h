#ifndef LOCOKIT_HW_INTERFACE_H
#define LOCOKIT_HW_INTERFACE_H

//ROS CONTROL
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

//ROS
#include <ros/ros.h>

//STANDARD
#include <signal.h>

//LOCOKIT INTERFACE
#include <LocoKitInterface.h>
#include <locokit/robot_configuration.h>




class LocokitHW : public hardware_interface::RobotHW
{
public:
  LocokitHW(ros::NodeHandle nh);
  bool configure();
  bool start();
  bool read();
  void write();
  void stop();

private:
  ros::NodeHandle nh_;
  bool tcp_connected_;
  LocoKitInterface* locokit_interface_;
  int sensor_number_;
  int motor_number_;
  int step_count_;
};

#endif // ifndef LOCOKIT_HW_INTERFACE_H

