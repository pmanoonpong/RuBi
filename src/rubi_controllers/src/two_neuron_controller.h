#ifndef TWO_NEURON_CPH_H
#define TWO_NEURON_CPH_H

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
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "dynamic_reconfigure/server.h"
#include "rubi_controllers/two_neuronConfig.h"
#include "gazebo_msgs/GetPhysicsProperties.h"

// GoRobots
#include "utils/ann-framework/ann.h"

/**
 * System signal
 */
bool killed(false);
void killerHandler(int sig) { killed = true; }

/**
 * @brief The TwoNeuronController class
 */
class TwoNeuronController {
 public:
  /**
   * @brief two_neuron_cph
   */
  TwoNeuronController();

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
  void callbackDynamicParameters(rubi_controllers::two_neuronConfig& config,
                                 uint32_t level);

  /**
   * @brief updateRate Updates the frequency at which the controller is updated.
   * This parameter is got from a default parameter from the launch file and, in
   * the
   * case of being used with gazebo, from the simulator perse.
   */
  void updateRate();

  /**
   * @brief step
   */
  void step();

  /**
   * @brief stop
   */
  void stop();

 private:
  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber sub_joint_states_, sub_left_foot_contact_,
      sub_right_foot_contact_;
  ros::Publisher pub_left_hip_, pub_left_knee_, pub_left_ankle_, pub_right_hip_,
      pub_right_knee_, pub_right_ankle_;
  ros::ServiceClient srv_client_gazebo_physic_properties_;

  std::string topic_left_hip_, topic_left_knee_, topic_left_ankle_,
      topic_right_hip_, topic_right_knee_, topic_right_ankle_,
      topic_joint_states_, topic_left_foot_contact_, topic_right_foot_contact_,
      topic_gazebo_physic_properties_;

  // Step time
  double time_step_;
  double real_time_factor_;
  double update_rate_;

  std::shared_ptr<ros::Rate> rate_;
  std::recursive_mutex update_rate_mutex_;
  std::thread update_rate_thread_;

  // Dynamic reconfigure
  typedef dynamic_reconfigure::Server<rubi_controllers::two_neuronConfig>
      DynamicReconfigServer;
  std::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
  DynamicReconfigServer::CallbackType param_reconfig_callback_;
  boost::recursive_mutex param_reconfig_mutex_;  // Must be boost

  // Controller: common
  double left_hip_pos_, left_knee_pos_, left_ankle_pos_, right_hip_pos_,
      right_knee_pos_, right_ankle_pos_;

  // Controller: ANN
  ANN ann_;
  double weight_w1_w1_, weight_w1_w2_, weight_w2_w1_, weight_w2_w2_, input1_,
      input2_;
};

#endif  // TWO_NEURON_CPH_H
