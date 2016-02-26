#include "two_neuron_controller.h"

TwoNeuronController::TwoNeuronController()
    : nh_("~"),
      time_step_(0.001),
      real_time_factor_(1000),
      update_rate_(10),
      ann_(2) {
  // Parameters
  // Get parameter names
  nh_.param<std::string>("joint_states_topic", topic_joint_states_,
                         "/dacbot/joint_states");
  nh_.param<std::string>("legs_topic", topic_legs_,
                         "/dacbot/leg_controllers/command");
  nh_.param<std::string>("left_foot_contact_topic", topic_left_foot_contact_,
                         "/dacbot/bumper/left_foot");
  nh_.param<std::string>("right_foot_contact_topic", topic_right_foot_contact_,
                         "/dacbot/bumper/right_foot");
  nh_.param<std::string>("gazebo_physic_properties_topic",
                         topic_right_foot_contact_,
                         "/gazebo/get_physics_properties");

  update_rate_mutex_.lock();
  rate_.reset(new ros::Rate(update_rate_ / (time_step_ * real_time_factor_)));
  update_rate_mutex_.unlock();

  update_rate_thread_ =
      std::thread(&TwoNeuronController::updateRate,
                  this);  // We do it in a different thread so we dont
                          // have to wait to the service call.
  update_rate_thread_.detach();

  // Dynamic parameters
  param_reconfig_server_.reset(
      new DynamicReconfigServer(param_reconfig_mutex_, nh_));

  dacbot_controller::two_neuronConfig config;
  param_reconfig_mutex_.lock();
  param_reconfig_server_->updateConfig(config.__getDefault__());
  param_reconfig_mutex_.unlock();

  param_reconfig_callback_ = boost::bind(
      &TwoNeuronController::callbackDynamicParameters, this, _1, _2);
  param_reconfig_server_->setCallback(param_reconfig_callback_);

  // Subcribers
  sub_joint_states_ = nh_.subscribe<sensor_msgs::JointState>(
      topic_joint_states_, 1, &TwoNeuronController::callbackSubcriberJointState,
      this);

  // Publishers
  pub_legs_ = nh_.advertise<std_msgs::Float64MultiArray>(topic_legs_, 1);

  // Starts ANN
  ann_.setWeight(0, 0, 1.4);
  ann_.setWeight(0, 1, 0.7);
  ann_.setWeight(1, 0, -0.7);
  ann_.setWeight(1, 1, 1.4);
  ann_.setBias(0, 0.01);
  ann_.setBias(1, 0.01);
  ann_.setInputScaling(0, 1);
  ann_.setInputScaling(1, 1);
}

void TwoNeuronController::callbackSubcriberJointState(
    sensor_msgs::JointState msg) {
  left_ankle_pos_ = msg.position.at(0);
  left_hip_pos_ = msg.position.at(1);
  left_knee_pos_ = msg.position.at(2);
  right_ankle_pos_ = msg.position.at(3);
  right_hip_pos_ = msg.position.at(4);
  right_knee_pos_ = msg.position.at(5);
}

void TwoNeuronController::callbackDynamicParameters(
    dacbot_controller::two_neuronConfig &config, uint32_t level) {
  weight_w1_w1_ = config.weight_self;
  weight_w1_w2_ = config.weight_other;
  weight_w2_w1_ = -config.weight_other;
  weight_w2_w2_ = config.weight_self;
  input1_ = config.input1_param;
  input2_ = config.input2_param;

  ann_.setWeight(0, 0, weight_w1_w1_);
  ann_.setWeight(0, 1, weight_w1_w2_);
  ann_.setWeight(1, 0, weight_w2_w1_);
  ann_.setWeight(1, 1, weight_w2_w2_);
  ann_.setInput(0, input1_);
  ann_.setInput(1, input2_);
}

void TwoNeuronController::step() {
  ann_.step();
  // ROS_INFO("%f, %f", ann_.getOutput(0), ann_.getOutput(1));

  // Depending on the enable, either reflexive signals or CPG-based are sent to
  // the motors
  std_msgs::Float64MultiArray
      motors_msg;  // (0) Left ankle, (1) Left knee, (2) Left hip,
                   // (3) Right ankle, (4) Right knee, (5) Right hip
  motors_msg.data.resize(6);

  // Knee adaptation
  double knee_output;
  //  (ann_.getOutput(1) <= 0)
  //      ? knee_output = 0
  //      : knee_output = ann_.getOutput(1);
  knee_output = ann_.getOutput(1);

  // (1) Left knee
  motors_msg.data.at(1) = -knee_output;
  // (2) Left hip
  motors_msg.data.at(2) = -ann_.getOutput(0);

  // (4) Left knee
  motors_msg.data.at(4) = knee_output;
  // (5) Left hip
  motors_msg.data.at(5) = ann_.getOutput(0);

  // Publish the message
  pub_legs_.publish(motors_msg);

  // Spin and update the rate
  ros::spinOnce();
  update_rate_mutex_.lock();
  rate_->sleep();
  update_rate_mutex_.unlock();
}

void TwoNeuronController::stop() {
  std_msgs::Float64MultiArray motors_msg;
  motors_msg.data.resize(6);

  for (int joint = 0; joint < 6; ++joint) motors_msg.data.at(joint) = 0.0;
  pub_legs_.publish(motors_msg);
  ROS_WARN("Two Neuron Controller stoped");
}

void TwoNeuronController::updateRate() {
  if (ros::service::exists(topic_gazebo_physic_properties_,
                           false)) {  // Checks if gazebo exists
    gazebo_msgs::GetPhysicsProperties msg;
    srv_client_gazebo_physic_properties_.call(msg);
    if (msg.response.time_step != time_step_ ||
        msg.response.max_update_rate != real_time_factor_) {
      ROS_INFO_STREAM("Time changed! Real Time Factor: "
                      << real_time_factor_ << " Time Step: " << time_step_);
      real_time_factor_ = msg.response.max_update_rate;
      time_step_ = msg.response.time_step;
      update_rate_mutex_.lock();
      rate_.reset(
          new ros::Rate(update_rate_ / (time_step_ * real_time_factor_)));
      update_rate_mutex_.unlock();
    }
  }
  std::this_thread::sleep_for(std::chrono::microseconds(50));
}
