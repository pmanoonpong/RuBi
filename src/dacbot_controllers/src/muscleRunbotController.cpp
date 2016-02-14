
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
  sub_joints_states = nh_.subscribe<sensor_msgs::JointState>(
      topic_joint_states_, 1,
      &MuscleRunbotController::callbackSubcriberJointState, this);

  // Publishers
  pub_left_ankle_ = nh_.advertise<std_msgs::Float64>(topic_left_ankle_, 1);
  pub_left_knee_ = nh_.advertise<std_msgs::Float64>(topic_left_knee_, 1);
  pub_left_hip_ = nh_.advertise<std_msgs::Float64>(topic_left_hip_, 1);
  pub_right_ankle_ = nh_.advertise<std_msgs::Float64>(topic_right_ankle_, 1);
  pub_right_knee_ = nh_.advertise<std_msgs::Float64>(topic_right_knee_, 1);
  pub_right_hip_ = nh_.advertise<std_msgs::Float64>(topic_right_hip_, 1);
}

void MuscleRunbotController::callbackSubcriberJointState(
    sensor_msgs::JointState& msg) {
  std::unique_lock<std::mutex> lock(joint_state_mutex_);
  left_ankle_pos_ = msg.position.at(0);
  left_hip_pos_ = msg.position.at(1);
  left_knee_pos_ = msg.position.at(2);
  right_ankle_pos_ = msg.position.at(3);
  right_hip_pos_ = msg.position.at(4);
  right_knee_pos_ = msg.position.at(5);
}

void MuscleRunbotController::init(int sensornumber, int motornumber,
                                  RandGen* randGen) {
  nSensors = sensornumber;
  nMotors = motornumber;
  steps = 0;

  hipPlot.open("/home/giuliano/Documents/thesis/plots/hips.dat");  // change
                                                                   // this to
                                                                   // your
                                                                   // directory

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

  freq.push_back(0);
  freq.push_back(0);
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

  actualAD = valarray<double>(sensornumber + 1);
  initialized = true;
}

int MuscleRunbotController::getSensorNumber() const { return nSensors; }

int MuscleRunbotController::getMotorNumber() const { return nMotors; }

void MuscleRunbotController::step(const sensor* sensors, int sensornumber,
                                  motor* motors, int motornumber) {
  valarray<double> motorOutput(4);
  steps++;

  actualAD[LEFT_HIP] = sensors[0];
  actualAD[RIGHT_HIP] = sensors[1];
  actualAD[LEFT_KNEE] = sensors[2];
  actualAD[RIGHT_KNEE] = sensors[3];
  actualAD[BOOM_ANGLE] = sensors[4];
  actualAD[LEFT_FOOT] = sensors[5];
  actualAD[RIGHT_FOOT] = sensors[6];
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

  // This is to simulate the feedback, you can cut the feedback to the CPG or
  // you can activate it in different periods

  // if((steps>3000 && steps<5000) || (steps >7000 && steps < 9000) || (steps
  // >11000 && steps < 13000) || (steps >15000 && steps < 19000))
  // sensora=0;

  // if(steps > 4000)
  //  sensora=0;

  std::vector<double> sign = DinLeft->generateOutputTwoLegThereshold(
      sensora, motors[0], motors[2], steps);  // generates motor signals
  DinLeft->setEnable(steps > 2000, motors[0], left_foot_sensor,
                     right_foot_sensor);  // generates enable output
  enable = DinLeft->getEnable();          // set enable
  std::cout << " enable: " << enable << std::endl;

  // wrtie to file, change directory
  if (steps > 500) {
    hipPlot << steps << " " << motorOutput[0] << " "
            << DinLeft->getCpg()->getFrequency() * 2.1 / 0.015 << " " << sign[0]
            << " " << motorOutput[0] << " " << motorOutput[2] << " "
            << motorOutput[3] << " " << actualAD[LEFT_HIP] << " "
            << actualAD[RIGHT_HIP] << " " << actualAD[LEFT_KNEE] << " "
            << actualAD[RIGHT_KNEE] << " " << actualAD[LEFT_FOOT] << " "
            << actualAD[RIGHT_FOOT] << std::endl;
  }

  // Depending on the enable, either reflexive signals or CPG-based are sent to
  // the motors
  motors[0] = motorOutput[0] * !enable + sign.at(0) * enable;  // LHMusclTor;
  motors[1] = motorOutput[1] * !enable + sign.at(1) * enable;  // RHMusclTor;
  motors[2] = motorOutput[2] * !enable + sign.at(2) * enable;  // LKMusclTor;
  motors[3] = motorOutput[3] * !enable + sign.at(3) * enable;  // RKMusclTor;
  motors[4] = ubc + ubc_wabl;

  if (steps % ubc_time == 0) ubc_wabl *= -1;

  // speed measurement..
  double stepsize = 0.01;  // length of control intervalls (both should be
                           // obtained automatically somehow..)
  double radius = 1;       // armlength in global coordinates
  if ((sensors[7] / 10 - pos) < -M_PI)
    speed = speed * 0.99 +
            0.01 * ((sensors[7] / 10 + 2 * M_PI - pos) * radius / 0.01);
  else
    speed = speed * 0.99 + 0.01 * ((sensors[7] / 10 - pos) * radius) / 0.01;
  pos = sensors[7] / 10;
}
