
#include "muscleRunbotController.h"

MuscleRunbotController::MuscleRunbotController() : nh_("~") {
  steps = 0;
  pos = 0;
  nSensors = 6;
  nMotors = 6;

  gait = new runbot::cGaitTransition(runbot::cGaitProfile());
  nnet = new runbot::cNNet((runbot::cGaitProfile*)gait);

  newGait = new runbot::cGaitProfile(78, 97, 115, 175, 2, 1.5);
  nnetTwo = new runbot::cNNet((runbot::cGaitProfile*)newGait);

  gait3 = new runbot::cGaitProfile(78, 115, 115, 175, 2, 2.7);
  nnet3 = new runbot::cNNet((runbot::cGaitProfile*)gait3);

  DinLeft = new DynamicCpg(0.04);   // 0.04
  DinRight = new DynamicCpg(0.04);  // 0.04

  actualAD = valarray<double>(6 + 1);

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
  nh_.param<std::string>("left_foot_contact_topic", topic_left_foot_contact_,
                         "/dacbot/bumper/left_foot");
  nh_.param<std::string>("right_foot_contact_topic", topic_right_foot_contact_,
                         "/dacbot/bumper/right_foot");

  // Subcribers
  sub_joint_states_ = nh_.subscribe<sensor_msgs::JointState>(
      topic_joint_states_, 1,
      &MuscleRunbotController::callbackSubcriberJointState, this);

  sub_left_foot_contact_ = nh_.subscribe<gazebo_msgs::ContactsState>(
      topic_left_foot_contact_, 1,
      &MuscleRunbotController::callbackSubcriberLeftFootContact, this);

  sub_right_foot_contact_ = nh_.subscribe<gazebo_msgs::ContactsState>(
      topic_right_foot_contact_, 1,
      &MuscleRunbotController::callbackSubcriberRightFootContact, this);

  // Publishers
  pub_left_ankle_ = nh_.advertise<std_msgs::Float64>(topic_left_ankle_, 1);
  pub_left_knee_ = nh_.advertise<std_msgs::Float64>(topic_left_knee_, 1);
  pub_left_hip_ = nh_.advertise<std_msgs::Float64>(topic_left_hip_, 1);
  pub_right_ankle_ = nh_.advertise<std_msgs::Float64>(topic_right_ankle_, 1);
  pub_right_knee_ = nh_.advertise<std_msgs::Float64>(topic_right_knee_, 1);
  pub_right_hip_ = nh_.advertise<std_msgs::Float64>(topic_right_hip_, 1);
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

void MuscleRunbotController::callbackSubcriberLeftFootContact(
    gazebo_msgs::ContactsState msg) {
  (msg.states.empty()) ? left_foot_contact_ = false : left_foot_contact_ = true;
}

void MuscleRunbotController::callbackSubcriberRightFootContact(
    gazebo_msgs::ContactsState msg) {
  (msg.states.empty()) ? right_foot_contact_ = false : right_foot_contact_ =
                                                           true;
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
      sensora, left_hip_effort_, left_knee_effort_, steps);  // generates motor signals
  DinLeft->setEnable(steps > 2000, left_hip_effort_, left_foot_contact_,
                     right_foot_contact_);  // generates enable output
  enable = DinLeft->getEnable();            // set enable
  std::cout << " enable: " << enable << std::endl;

  // Depending on the enable, either reflexive signals or CPG-based are sent to
  // the motors
  std_msgs::Float64 motor_msg;

  left_hip_effort_ = motorOutput[0] * !enable + sign.at(0) * enable;  // LHMusclTor;
  motor_msg.data = left_hip_effort_;
  pub_left_hip_.publish(motor_msg);

  right_hip_effort_ = motorOutput[1] * !enable + sign.at(1) * enable;  // RHMusclTor;
  motor_msg.data = right_hip_effort_;
  pub_right_hip_.publish(motor_msg);

  left_knee_effort_ = motorOutput[2] * !enable + sign.at(2) * enable;  // LKMusclTor;
  motor_msg.data = left_knee_effort_;
  pub_left_knee_.publish(motor_msg);

  right_knee_effort_ = motorOutput[3] * !enable + sign.at(3) * enable;  // RKMusclTor;
  motor_msg.data = right_knee_effort_;
  pub_right_knee_.publish(motor_msg);

  //motors[4] = ubc + ubc_wabl;

  if (steps % ubc_time == 0) ubc_wabl *= -1;

  // speed measurement..
  /*
  double radius = 1;  // armlength in global coordinates
  if ((sensors[7] / 10 - pos) < -M_PI)
    speed = speed * 0.99 +
            0.01 * ((sensors[7] / 10 + 2 * M_PI - pos) * radius / 0.01);
  else
    speed = speed * 0.99 + 0.01 * ((sensors[7] / 10 - pos) * radius) / 0.01;
  pos = sensors[7] / 10;
  */
}
