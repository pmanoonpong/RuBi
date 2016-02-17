/*
 * muscleRunbotController.h
 *
 *  Created on: 08.03.2014
 *      Author: Johannes Widenka
 */

#ifndef MUSCLERUNBOTCONTROLLER_H_
#define MUSCLERUNBOTCONTROLLER_H_

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

// GOrobots
#include "cgaittransition.h"
#include "cnnet.h"
#include "DynamicCpg.h"

/// Channel number description
const unsigned char BOOM_ANGLE = 1;
const unsigned char LEFT_FOOT = 2;
const unsigned char RIGHT_FOOT = 3;
const unsigned char LEFT_HIP = 4;
const unsigned char RIGHT_HIP = 5;
const unsigned char LEFT_KNEE = 6;
const unsigned char RIGHT_KNEE = 7;

class MuscleRunbotController {
 public:
  /**
   * @brief MuscleRunbotController Default constructor
   */
  MuscleRunbotController();

  /**
   * @brief Stop the legs and destroyes the node
   */
  ~MuscleRunbotController();

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

 private:
  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber sub_joint_states_, sub_left_foot_contact_, sub_right_foot_contact_;
  ros::Publisher pub_left_hip_, pub_left_knee_, pub_left_ankle_, pub_right_hip_,
      pub_right_knee_, pub_right_ankle_;

  std::string topic_left_hip_, topic_left_knee_, topic_left_ankle_,
      topic_right_hip_, topic_right_knee_, topic_right_ankle_,
      topic_joint_states_, topic_left_foot_contact_, topic_right_foot_contact_;

  // Controller
  float left_hip_pos_, left_knee_pos_, left_ankle_pos_, right_hip_pos_,
      right_knee_pos_, right_ankle_pos_;

  bool left_foot_contact_, right_foot_contact_;

  double left_hip_effort_, left_knee_effort_, left_ankle_effort_, right_hip_effort_,
  right_knee_effort_, right_ankle_effort_;

  // Mutex
  std::mutex joint_state_mutex_;

  // Controller
  int nSensors;
  int nMotors;
  int steps;             // counter for controll steps
  double ubc;            // position of the upper body component (-1..+1)
  double speed;          // current speed of the robot
  double pos;            // current position of the robot

  double ubc_wabl =
      0.0;  // movement of the UBC around its position variable "ubc"
  // wabbling from ubc-ubc_wabl to ubc+ubc_wabl
  int ubc_time = 100;   // sec/100   -   defines the time where the ubc changes
                        // its movement direction
  runbot::cNNet* nnet;  // ANN controlling the movement of the robot
  runbot::cGaitTransition* gait;  // gait parameter for the ANN
  runbot::cNNet* nnetTwo;
  runbot::cGaitTransition* gait_two;
  runbot::cGaitProfile* gait3;
  runbot::cGaitProfile* newGait;
  runbot::cNNet* nnet3;

  valarray<double>
      actualAD;  // array, used for mapping the sensor array to the right order

  DynamicCpg* DinLeft, *DinRight;

};

/**
 * @brief Runs the controller
 */
int main() {
  // Init
  int argc(0);
  char** argv(NULL);
  ros::init(argc, argv, "muscle_controller");

  // Controller
  MuscleRunbotController muscleRunbotController;

  // Rate
  ros::Rate rate(10);

  // ROS Spin: Handle callbacks
  while (!ros::isShuttingDown()) {
    muscleRunbotController.step();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

#endif /* MUSCLERUNBOTCONTROLLER_H_ */
