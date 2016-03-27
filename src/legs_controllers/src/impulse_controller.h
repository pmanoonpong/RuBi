#ifndef IMPULSE_CONTROLLER_H
#define IMPULSE_CONTROLLER_H

// System
#include <signal.h>

// STD
#include <vector>
#include <cmath>
#include <iostream>
#include <string>
#include <mutex>
#include <thread>

// ROS
#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/GetPhysicsProperties.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "std_srvs/Empty.h"
#include "legs_controllers/impulse.h"
#include "legs_controllers/impulse_controllerConfig.h"

#define DEG_TO_RAD M_PI / 180
#define RAD_TO_DEG 180 / M_PI

// Defined by the protocol of the robot
enum MOTORS {
  LEFT_ANKLE = 0,
  LEFT_KNEE = 1,
  LEFT_HIP = 2,
  RIGHT_ANKLE = 3,
  RIGHT_KNEE = 4,
  RIGHT_HIP = 5
};

class ImpulseController {
 public:
  ImpulseController();

  /**
   * @brief updateRate Updates the frequency at which the controller is updated.
   * This parameter is got from a default parameter from the launch file and, in
   * the case of being used with gazebo, from the simulator perse.
   */
  void updateRate();

  /**
   * @brief step
   */
  void step();

  /**
   * @brief restart The simulation and puts the robot in the inital position
   */
  void resetSimulation();

  // Callbacks
  /**
   * @brief callbackSubcriberJointState
   * @param msg
   */
  void callbackSubcriberJointState(sensor_msgs::JointState msg);

  /**
   * @brief callbackDynamicParameters
   * @param config
   * @param level
   */
  void callbackDynamicParameters(
      legs_controller::impulse_controllerConfig& config, uint32_t level);

  /**
   * @brief callbackServiceImpulse
   */
  bool callbackServiceImpulse(legs_controllers::impulse::Request& req,
                              legs_controllers::impulse::Response& res);

 private:
  // ROS
  ros::NodeHandle nh_;
  ros::Publisher pub_legs_;
  ros::Subscriber sub_joint_states_;
  ros::ServiceClient srv_client_gazebo_physic_properties_,
      srv_client_gazebo_set_model_configuration_, srv_client_reset_simulation;

  ros::ServiceServer srv_server_impulse_;

  std::string topic_legs_, topic_joint_states_, topic_gazebo_physic_properties_,
      topic_gazebo_set_model_configuration_, topic_gazebo_reset_simulation_,
      topic_impulse_;

  // Step time
  double time_step_;
  double real_time_factor_;
  double update_rate_;
  std::shared_ptr<ros::Rate> rate_;
  std::recursive_mutex update_rate_mutex_;
  std::thread update_rate_thread_;

  // Dynamic reconfigure
  typedef dynamic_reconfigure::Server<legs_controller::impulse_controllerConfig>
      DynamicReconfigServer;
  std::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
  DynamicReconfigServer::CallbackType param_reconfig_callback_;
  boost::recursive_mutex param_reconfig_mutex_;  // Must be boost

  // Controller: common
  double left_hip_pos_, left_knee_pos_, left_ankle_pos_, right_hip_pos_,
      right_knee_pos_, right_ankle_pos_;

  double left_hip_initial_pos_, left_knee_initial_pos_, left_ankle_initial_pos_,
      right_hip_initial_pos_, right_knee_initial_pos_, right_ankle_initial_pos_;
};

#endif  // IMPULSE_CONTROLLER_H
