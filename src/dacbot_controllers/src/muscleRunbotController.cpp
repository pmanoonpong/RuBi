
#include "muscleRunbotController.h"

MuscleRunbotController::MuscleRunbotController() : nh_("~") {
  steps = 0;
  pos = 0;
  nSensors = 0;
  nMotors = 0;
  simulatedMass = 0.4;
  gait = new runbot::cGaitTransition(runbot::cGaitProfile());
  nnet = new runbot::cNNet((runbot::cGaitProfile*)gait);

  newGait = new runbot::cGaitProfile(78, 97, 115, 175, 2, 1.5);
  nnetTwo = new runbot::cNNet((runbot::cGaitProfile*)newGait);

  gait3 = new runbot::cGaitProfile(78, 115, 115, 175, 2, 2.7);
  nnet3 = new runbot::cNNet((runbot::cGaitProfile*)gait3);

  initialized = false;

  // Parameters
  // Get parameter names
  nh_.param<std::string>("joint_states_topic", topic_joint_states_,
                         "/dacbot/joint_states");
  nh_.param<std::string>("left_hip_topic", topic_left_hip_,
                         "/dacbot/left_hip_effort/command");
  nh_.param<std::string>("left_knee_topic", topic_left_knee_,
                         "/dacbot/left_knee_effort/command");
  nh_.param<std::string>("left_ankle_topic", topic_left_ankle_,
                         "/dacbot/left_ankle_effort/command");
  nh_.param<std::string>("right_hip_topic", topic_right_hip_,
                         "/dacbot/right_hip_effort/command");
  nh_.param<std::string>("right_knee_topic", topic_right_knee_,
                         "/dacbot/right_knee_effort/command");
  nh_.param<std::string>("right_ankle_topic", topic_right_ankle_,
                         "/dacbot/right_ankle_effort/command");

  // Subcribers
  sub_joint_states_ = nh_.subscribe<sensor_msgs::JointState>(
      topic_joint_states_, 1,
      &MuscleRunbotController::callbackSubcriberJointState, this);

  // Publishers
  pub_left_ankle_ = nh_.advertise<std_msgs::Float64>(topic_left_ankle_, 1);
  pub_left_knee_ = nh_.advertise<std_msgs::Float64>(topic_left_knee_, 1);
  pub_left_hip_ = nh_.advertise<std_msgs::Float64>(topic_left_hip_, 1);
  pub_right_ankle_ = nh_.advertise<std_msgs::Float64>(topic_right_ankle_, 1);
  pub_right_knee_ = nh_.advertise<std_msgs::Float64>(topic_right_knee_, 1);
  pub_right_hip_ = nh_.advertise<std_msgs::Float64>(topic_right_hip_, 1);

  nSensors = 6;
  nMotors = 6;
  steps = 0;

  motor0DerivativeVector.push_back(0);
  motor0DerivativeVector.push_back(0);

  leftHipDerivativeVector.push_back(0);
  leftHipDerivativeVector.push_back(0);

  leftDerivativeVector.push_back(0);
  leftDerivativeVector.push_back(0);

  rightDerivativeVector.push_back(0);
  rightDerivativeVector.push_back(0);

  systemFrequencyVector.push_back(0);
  systemFrequencyVector.push_back(0);

  shiftVector.push_back(0);
  shiftVector.push_back(0);

  freqDeriv.push_back(0);
  freqDeriv.push_back(0);

  stepFreq.push_back(0);
  stepFreq.push_back(0);

  frequencySystem = 0;

  DinLeft = new DynamicCpg(0.04);   // 0.04
  DinRight = new DynamicCpg(0.04);  // 0.04

  filter = new lowPass_filter(0.2);
  phase = new shift_register(3);  // 4
  rightHipDelayed = new shift_register(0);

  leftKneeDelayed = new shift_register(6);
  rightKneeDelayed = new shift_register(4);

  actualAD = valarray<double>(6 + 1);
  initialized = true;
}

void MuscleRunbotController::callbackSubcriberJointState(
    sensor_msgs::JointState msg) {
  std::unique_lock<std::mutex> lock(joint_state_mutex_);
  left_ankle_pos_ = msg.position.at(0);
  left_hip_pos_ = msg.position.at(1);
  left_knee_pos_ = msg.position.at(2);
  right_ankle_pos_ = msg.position.at(3);
  right_hip_pos_ = msg.position.at(4);
  right_knee_pos_ = msg.position.at(5);
}

void MuscleRunbotController::step() {
  valarray<double> motorOutput(4);
  steps++;

  actualAD[LEFT_HIP] = left_hip_pos_;
  actualAD[RIGHT_HIP] = right_hip_pos_;
  actualAD[LEFT_KNEE] = left_knee_pos_;
  actualAD[RIGHT_KNEE] = right_knee_pos_;
  actualAD[BOOM_ANGLE] = 0;
  actualAD[LEFT_FOOT] = left_ankle_pos_;
  actualAD[RIGHT_FOOT] = right_ankle_pos_;
  actualAD[0] = steps;

  double enable;
  std::cout << steps << std::endl;

  double sensora;
  double left_foot_sensor = (sensors[6] > 3000) ? 0.0 : 1.0;
  double right_foot_sensor = (sensors[5] > 3000) ? 0.0 : 1.0;

  // Used to generate 1 gait with reflexive nn
  // motorOutput=nnet->update(actualAD,steps);

  // This is used to generate different gaits with the reflexive NN, change
  // according to your experiment
  if (steps < 5000)
    motorOutput = nnet->update(actualAD, steps);
  else if (steps >= 5000 && steps < 7000)
    motorOutput = nnetTwo->update(actualAD, steps);
  else
    motorOutput = nnet3->update(actualAD, steps);

  sensora = actualAD[LEFT_HIP];  // feedback

  std::vector<double> sign = DinLeft->generateOutputTwoLegThereshold(
      sensora, motors[0], motors[2], steps);  // generates motor signals
  DinLeft->setEnable(steps > 2000, motors[0], left_foot_sensor,
                     right_foot_sensor);  // generates enable output
  enable = DinLeft->getEnable();          // set enable
  std::cout << " enable: " << enable << std::endl;

  // Depending on the enable, either reflexive signals or CPG-based are sent to
  // the motors
  motors[0] = motorOutput[0] * !enable + sign.at(0) * enable;  // LHMusclTor;
  motors[1] = motorOutput[1] * !enable + sign.at(1) * enable;  // RHMusclTor;
  motors[2] = motorOutput[2] * !enable + sign.at(2) * enable;  // LKMusclTor;
  motors[3] = motorOutput[3] * !enable + sign.at(3) * enable;  // RKMusclTor;
  motors[4] = ubc + ubc_wabl;

  if (steps % ubc_time == 0) ubc_wabl *= -1;

  // speed measurement..
  double radius = 1;       // armlength in global coordinates
  if ((sensors[7] / 10 - pos) < -M_PI)
    speed = speed * 0.99 +
            0.01 * ((sensors[7] / 10 + 2 * M_PI - pos) * radius / 0.01);
  else
    speed = speed * 0.99 + 0.01 * ((sensors[7] / 10 - pos) * radius) / 0.01;
  pos = sensors[7] / 10;
}
