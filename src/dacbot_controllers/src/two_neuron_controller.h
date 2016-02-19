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

// ROS
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/ContactsState.h"

// GoRobots
#include "utils/ann-framework/ann.h"

/**
 * System signal
 */
bool killed(false);
void killerHandler(int sig) {
  killed = true;
}

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

  void callbackSubcriberLeftFootContact(gazebo_msgs::ContactsState msg);

  void callbackSubcriberRightFootContact(gazebo_msgs::ContactsState msg);

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

  std::string topic_left_hip_, topic_left_knee_, topic_left_ankle_,
      topic_right_hip_, topic_right_knee_, topic_right_ankle_,
      topic_joint_states_, topic_left_foot_contact_, topic_right_foot_contact_;

  // Controller: common
  double left_hip_pos_, left_knee_pos_, left_ankle_pos_, right_hip_pos_,
      right_knee_pos_, right_ankle_pos_;

  bool left_foot_contact_, right_foot_contact_;

  double left_hip_effort_, left_knee_effort_, left_ankle_effort_,
      right_hip_effort_, right_knee_effort_, right_ankle_effort_;

  std::mutex joint_state_mutex_;

  // Controller: ANN
  ANN ann_;
};

/**
 * @brief Runs the controller
 */
int main() {
  // Init
  int argc(0);
  char** argv(NULL);
  ros::init(argc, argv, "two_neuron_controller", ros::init_options::NoSigintHandler);

  // Controller
  TwoNeuronController twoNeuronController;

  // Signal
  signal(SIGINT, killerHandler);

  // Rate
  ros::Rate rate(10);

  // ROS Spin: Handle callbacks
  while (!killed) {
    twoNeuronController.step();
    ros::spinOnce();
    rate.sleep();
  }

  twoNeuronController.stop();
  ros::shutdown();

  return 0;
}

#endif  // TWO_NEURON_CPH_H
