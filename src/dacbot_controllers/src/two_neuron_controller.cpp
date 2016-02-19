#include "two_neuron_controller.h"

TwoNeuronController::TwoNeuronController() : ann_(2) {
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
      &TwoNeuronController::callbackSubcriberJointState, this);

  sub_left_foot_contact_ = nh_.subscribe<gazebo_msgs::ContactsState>(
      topic_left_foot_contact_, 1,
      &TwoNeuronController::callbackSubcriberLeftFootContact, this);

  sub_right_foot_contact_ = nh_.subscribe<gazebo_msgs::ContactsState>(
      topic_right_foot_contact_, 1,
      &TwoNeuronController::callbackSubcriberRightFootContact, this);

  // Publishers
  pub_left_ankle_ = nh_.advertise<std_msgs::Float64>(topic_left_ankle_, 1);
  pub_left_knee_ = nh_.advertise<std_msgs::Float64>(topic_left_knee_, 1);
  pub_left_hip_ = nh_.advertise<std_msgs::Float64>(topic_left_hip_, 1);
  pub_right_ankle_ = nh_.advertise<std_msgs::Float64>(topic_right_ankle_, 1);
  pub_right_knee_ = nh_.advertise<std_msgs::Float64>(topic_right_knee_, 1);
  pub_right_hip_ = nh_.advertise<std_msgs::Float64>(topic_right_hip_, 1);

  // Oscilator
  ann_.setDefaultTransferFunction(ann_.tanhFunction());

  // Update ann weights
  ann_.setWeight(0, 0, 1.5);
  ann_.setWeight(0, 1, 0.4);
  ann_.setWeight(1, 0, -0.4);
  ann_.setWeight(1, 1, 1.5);

}

void TwoNeuronController::callbackSubcriberJointState(
    sensor_msgs::JointState msg) {
  std::unique_lock<std::mutex> lock(joint_state_mutex_);
  left_ankle_pos_ = msg.position.at(0);
  left_hip_pos_ = msg.position.at(1);
  left_knee_pos_ = msg.position.at(2);
  right_ankle_pos_ = msg.position.at(3);
  right_hip_pos_ = msg.position.at(4);
  right_knee_pos_ = msg.position.at(5);
}

void TwoNeuronController::callbackSubcriberLeftFootContact(
    gazebo_msgs::ContactsState msg) {
  (msg.states.empty()) ? left_foot_contact_ = false : left_foot_contact_ = true;
}

void TwoNeuronController::callbackSubcriberRightFootContact(
    gazebo_msgs::ContactsState msg) {
  (msg.states.empty()) ? right_foot_contact_ = false : right_foot_contact_ =
                                                           true;
}

void TwoNeuronController::step() {


  // Depending on the enable, either reflexive signals or CPG-based are sent to
  // the motors
  std_msgs::Float64 motor_msg;

  // Left hip
  motor_msg.data = ann_.getActivity(ann_.getNeuron(0));
  pub_left_hip_.publish(motor_msg);

  // Right hip
  motor_msg.data = -ann_.getActivity(ann_.getNeuron(0));
  pub_right_hip_.publish(motor_msg);

  // Left knee
  motor_msg.data = -ann_.getActivity(ann_.getNeuron(1));
  pub_left_knee_.publish(motor_msg);

  // Right knee
  motor_msg.data = ann_.getActivity(ann_.getNeuron(1));
  pub_right_knee_.publish(motor_msg);
}
