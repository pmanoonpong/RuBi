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
                         "/rubi/joint_states");
  nh_.param<std::string>("left_hip_topic", topic_left_hip_,
                         "/rubi/left_hip_effort/command");
  nh_.param<std::string>("left_knee_topic", topic_left_knee_,
                         "/rubi/left_knee_effort/command");
  nh_.param<std::string>("left_ankle_topic", topic_left_ankle_,
                         "/rubi/left_ankle_effort/command");
  nh_.param<std::string>("right_hip_topic", topic_right_hip_,
                         "/rubi/right_hip_effort/command");
  nh_.param<std::string>("right_knee_topic", topic_right_knee_,
                         "/rubi/right_knee_effort/command");
  nh_.param<std::string>("right_ankle_topic", topic_right_ankle_,
                         "/rubi/right_ankle_effort/command");
  nh_.param<std::string>("left_foot_contact_topic", topic_left_foot_contact_,
                         "/rubi/bumper/left_foot");
  nh_.param<std::string>("right_foot_contact_topic", topic_right_foot_contact_,
                         "/rubi/bumper/right_foot");
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

  rubi_controllers::two_neuronConfig config;
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
  pub_left_ankle_ = nh_.advertise<std_msgs::Float64>(topic_left_ankle_, 1);
  pub_left_knee_ = nh_.advertise<std_msgs::Float64>(topic_left_knee_, 1);
  pub_left_hip_ = nh_.advertise<std_msgs::Float64>(topic_left_hip_, 1);
  pub_right_ankle_ = nh_.advertise<std_msgs::Float64>(topic_right_ankle_, 1);
  pub_right_knee_ = nh_.advertise<std_msgs::Float64>(topic_right_knee_, 1);
  pub_right_hip_ = nh_.advertise<std_msgs::Float64>(topic_right_hip_, 1);

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
    rubi_controllers::two_neuronConfig& config, uint32_t level) {
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
  std_msgs::Float64 motor_msg;

  double knee_output;
  double hip_output;
  double multiplier(1);
  //(ann_.getOutput(1) <= 0) ? knee_output = 0
  //                         : knee_output = ann_.getOutput(1) * multiplier;
  knee_output = ann_.getOutput(1) * multiplier;
  hip_output = ann_.getOutput(0) * multiplier;

  // Left hip
  motor_msg.data = -hip_output;
  pub_left_hip_.publish(motor_msg);

  // Right hip
  motor_msg.data = hip_output;
  pub_right_hip_.publish(motor_msg);

  // Left knee
  motor_msg.data = knee_output;
  pub_left_knee_.publish(motor_msg);

  // Right knee
  motor_msg.data = -knee_output;
  pub_right_knee_.publish(motor_msg);

  // Spin and update the rate
  ros::spinOnce();
  update_rate_mutex_.lock();
  rate_->sleep();
  update_rate_mutex_.unlock();
}

void TwoNeuronController::stop() {
  std_msgs::Float64 motor_msg;
  motor_msg.data = 0.0;

  // Left hip
  pub_left_hip_.publish(motor_msg);
  // Right hip
  pub_right_hip_.publish(motor_msg);
  // Left knee
  pub_left_knee_.publish(motor_msg);
  // Right knee
  pub_right_knee_.publish(motor_msg);
  ROS_WARN("Two Neuron Controller stoped");
}

void TwoNeuronController::updateRate() {
  while (!killed) {
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
}

/**
 * @brief Runs the controller
 */
int main() {
  // Init
  int argc(0);
  char** argv(NULL);
  ros::init(argc, argv, "two_neuron_controller",
            ros::init_options::NoSigintHandler);

  // Controller
  TwoNeuronController twoNeuronController;

  // Signal
  signal(SIGINT, killerHandler);

  // Step until killed
  while (!killed) {
    twoNeuronController.step();
  }

  twoNeuronController.stop();
  ros::shutdown();

  return 0;
}
