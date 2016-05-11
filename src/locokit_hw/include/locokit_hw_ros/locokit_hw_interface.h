#ifndef LOCOKIT_HW_INTERFACE_H
#define LOCOKIT_HW_INTERFACE_H

//ROS
#include <ros/ros.h>

//ROS CONTROL
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

//STANDARD
#include <signal.h>
#include <time.h>
#include <string>
#include <map>
#include <vector>
#include <chrono>
#include <thread>

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
  bool write();
  virtual ~LocokitHW();

private:
  ros::NodeHandle nh_;

  LocoKitInterface* locokit_interface_;
  bool tcp_connected_;
  int step_count_;
  int sensor_number_;
  int motor_number_;

  hardware_interface::JointStateInterface joint_state_interface_;
  //hardware_interface::JointCommandInterface joint_command_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;
  motor motors_[locokitMotor::NUMBER_MOTORS];
  motor motors_prev_[locokitMotor::NUMBER_MOTORS];
  sensor sensors_[locokitSensor::NUMBER_SENSORS];
  double vel[locokitSensor::NUMBER_SENSORS];
  double eff[locokitSensor::NUMBER_SENSORS];
  std::map<int, std::string> joint_names;

};

#endif // ifndef LOCOKIT_HW_INTERFACE_H

