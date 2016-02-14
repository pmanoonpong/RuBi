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

// ROS
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#include "cgaittransition.h"
#include "cnnet.h"
#include "plastic.h"
#include "DynamicCpg.h"
#include "lowPassfilter.h"
#include "controllers/runbotii_dacbot/shiftregister.h"
#include "controllers/runbotii_dacbot/derivativeTransitionRegister.h"

#include "utils/vaam-library/musclechain.h"
#include "utils/vaam-library/dccontrollingvmm.h"

/// Channel number description
const int BOOM_ANGLE = 1;
const int LEFT_FOOT =
    2;  // = 0 .. -4.96v,  2048 = 0V (foot contact the ground) ....
        //                4096 = + 4.96V (foot off ground)
const int RIGHT_FOOT = 3;  // chanel 3
const int LEFT_HIP = 4;
const int RIGHT_HIP = 5;
const int LEFT_KNEE = 6;
const int RIGHT_KNEE = 7;

class MuscleRunbotController {
 public:
  MuscleRunbotController();

  void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  void step(const sensor* sensors, int sensornumber, motor* motors,
            int motornumber);

  // Callbacks
  void callbackSubcriberJointState(std_msgs::Float64MultiArray &msg);

 private:
  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber sub_joint_states_;
  ros::Publisher pub_left_hip_, pub_left_knee_, pub_left_ankle_, pub_right_hip_,
      pub_right_knee_, pub_right_ankle_;

  std::string topic_left_hip_, topic_left_knee_, topic_left_ankle_, topic_right_hip_,
  topic_right_knee_, topic_right_ankle_, topic_joint_states_;

  // Controller
  int nSensors;
  int nMotors;
  int steps;             // counter for controll steps
  double ubc;            // position of the upper body component (-1..+1)
  double speed;          // current speed of the robot
  double pos;            // current position of the robot
  double simulatedMass;  // simulated mass of the robot, used by all modelled
                         // muscles
  double ubc_wabl =
      0.0;  // movement of the UBC around its position variable "ubc"
  // wabbling from ubc-ubc_wabl to ubc+ubc_wabl
  int ubc_time = 100;   // sec/100   -   defines the time where the ubc changes
                        // its movement direction
  runbot::cNNet* nnet;  // ANN controlling the movement of the robot
  runbot::cNNet* nnetTwo;
  runbot::cGaitTransition* gait;  // gait parameter for the ANN
  runbot::cGaitTransition* gait_two;
  runbot::cGaitProfile* newGait;
  runbot::cGaitProfile* gait3;
  runbot::cNNet* nnet3;
  DCControllingVMM* RHmuscles;  // modelled muscles for each joint..
  DCControllingVMM* LHmuscles;
  DCControllingVMM* RKmuscles;
  DCControllingVMM* LKmuscles;

  // muscle chains that handle the communication for muscles depending on each
  // other
  MuscleChain* leftMuscles;
  MuscleChain* rightMuscles;

  valarray<double>
      actualAD;  // array, used for mapping the sensor array to the right order

  bool initialized = false;
  // giuliano

  std::ofstream hipPlot;  // plot
  std::ofstream cpgLeft;
  std::ofstream cpgPlot;
  std::ofstream cpgRight;

  double sensorFeedback;
  double feetFeedback;
  lowPass_filter* filter;
  std::vector<double> leftDerivativeVector, rightDerivativeVector,
      motor0DerivativeVector, leftHipDerivativeVector, freqDeriv;
  double cpg_right_hip = 0;
  double cpg_left_hip = 0;
  std::vector<double> stepFreq;
  double frequencySystem;
  double cpg_left_knee = 0;
  double cpg_right_knee = 0;
  int getShiftDelay(double out1, double out2, int step);
  int countDelay = 0;
  std::vector<double> shiftVector, derOut1, derOut2;
  double cpgcounter = 0;

  std::vector<double> systemFrequencyVector;
  // signal parameters//
  double amplitudeHips;
  double amplitudeKnee;
  double max;
  double min = 1000;
  double controllerEnable = 0;
  std::vector<double> freq;
  // signal parameters
  double cpg_signal_right;
  double cpg_signal_left;
  double perturbation;
  bool cpgNoPerturbation;

  double errorVal = 0, errorCount = 0;

  bool oneCPG = true;
  DynamicCpg* DinLeft, *DinRight;

  std::vector<double> shift;
  shift_register* phase, *leftKneeDelayed, *rightKneeDelayed, *leftHipDelayed,
      *rightHipDelayed;
  derivativeTransitionRegister* checkWave;
};

#endif /* MUSCLERUNBOTCONTROLLER_H_ */
