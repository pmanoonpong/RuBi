#include "impulse_controller.h"

/**
 * System signal
 */
bool killed(false);
void killerHandler(int sig) { killed = true; }
/**
 * @brief Runs the controller
 */
int main() {
  // Init
  int argc(0);
  char **argv(NULL);
  ros::init(argc, argv, "impulse_controller",
            ros::init_options::NoSigintHandler);

  // Controller
  ImpulseController impulseController;

  // Signal
  signal(SIGINT, killerHandler);

  // Step until killed
  while (!killed) {
    impulseController.step();
  }

  ros::shutdown();

  return 0;
}

ImpulseController::ImpulseController()
    : nh_("~"), time_step_(0.001), real_time_factor_(1000), update_rate_(10) {
  // Parameters
  // Get parameter names
  nh_.param<std::string>("joint_states_topic", topic_joint_states_,
                         "/legs/joint_states");
  nh_.param<std::string>("legs_topic", topic_legs_,
                         "/legs/leg_controllers/command");
  nh_.param<std::string>("gazebo_physic_properties_topic",
                         topic_gazebo_physic_properties_,
                         "/gazebo/get_physics_properties");
  nh_.param<std::string>("gazebo_set_model_configuration_topic",
                         topic_gazebo_set_model_configuration_,
                         "/gazebo/set_model_configuration");
  nh_.param<std::string>("gazebo_reset_simulation_topic",
                         topic_gazebo_reset_simulation_,
                         "/gazebo/reset_simulation");
  nh_.param<std::string>("impulse_topic", topic_impulse_, "/legs/impulse");

  // Get update rate from gazebo
  update_rate_mutex_.lock();
  rate_.reset(new ros::Rate(update_rate_ / (time_step_ * real_time_factor_)));
  update_rate_mutex_.unlock();

  update_rate_thread_ =
      std::thread(&ImpulseController::updateRate,
                  this);  // We do it in a different thread so we dont
                          // have to wait to the service call.
  update_rate_thread_.detach();

  // Dynamic parameters
  param_reconfig_server_.reset(
      new DynamicReconfigServer(param_reconfig_mutex_, nh_));

  legs_controller::impulse_controllerConfig config;
  param_reconfig_mutex_.lock();
  param_reconfig_server_->updateConfig(config.__getDefault__());
  param_reconfig_mutex_.unlock();

  param_reconfig_callback_ =
      boost::bind(&ImpulseController::callbackDynamicParameters, this, _1, _2);
  param_reconfig_server_->setCallback(param_reconfig_callback_);

  // Publishers
  pub_legs_ = nh_.advertise<std_msgs::Float64MultiArray>(topic_legs_, 1);

  // Services
  srv_server_impulse_ = nh_.advertiseService(
      topic_impulse_, &ImpulseController::callbackServiceImpulse, this);

  srv_client_reset_simulation =
      nh_.serviceClient<std_srvs::Empty>(topic_gazebo_reset_simulation_);
  srv_client_gazebo_physic_properties_ =
      nh_.serviceClient<gazebo_msgs::GetPhysicsProperties>(
          topic_gazebo_physic_properties_);
  srv_client_gazebo_set_model_configuration_ =
      nh_.serviceClient<gazebo_msgs::SetModelConfiguration>(
          topic_gazebo_set_model_configuration_);
}

void ImpulseController::step() {
  // std_msgs::Float64MultiArray
  //    motors_msg;  // (0) Left ankle, (1) Left knee, (2) Left hip,
  // (3) Right ankle, (4) Right knee, (5) Right hip
  // motors_msg.data.resize(6);

  // Publish the message
  // pub_legs_.publish(motors_msg);

  // Spin and update the rate
  ros::spinOnce();
  update_rate_mutex_.lock();
  rate_->sleep();
  update_rate_mutex_.unlock();
}

void ImpulseController::resetSimulation() {
  // Resets the simulation
  ROS_INFO("Reset");
  std_srvs::Empty reset_simualtion_srv;
  srv_client_reset_simulation.call(reset_simualtion_srv);

  // Sets initial positions
  gazebo_msgs::SetModelConfiguration initial_configuration_msg;
  initial_configuration_msg.request.model_name = "legs";
  initial_configuration_msg.request.urdf_param_name = "robot_description";

  initial_configuration_msg.request.joint_names.push_back("left_hip");
  initial_configuration_msg.request.joint_positions.push_back(
      left_hip_initial_pos_);
  initial_configuration_msg.request.joint_names.push_back("left_knee");
  initial_configuration_msg.request.joint_positions.push_back(
      left_knee_initial_pos_);
  initial_configuration_msg.request.joint_names.push_back("left_ankle");
  initial_configuration_msg.request.joint_positions.push_back(
      left_ankle_initial_pos_);

  initial_configuration_msg.request.joint_names.push_back("right_hip");
  initial_configuration_msg.request.joint_positions.push_back(
      right_hip_initial_pos_);
  initial_configuration_msg.request.joint_names.push_back("right_knee");
  initial_configuration_msg.request.joint_positions.push_back(
      right_knee_initial_pos_);
  initial_configuration_msg.request.joint_names.push_back("right_ankle");
  initial_configuration_msg.request.joint_positions.push_back(
      right_ankle_initial_pos_);

  srv_client_gazebo_set_model_configuration_.call(initial_configuration_msg);
}

void ImpulseController::updateRate() {
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

void ImpulseController::callbackDynamicParameters(
    legs_controller::impulse_controllerConfig &config, uint32_t level) {
  left_ankle_initial_pos_ = config.left_ankle_initial_pos * DEG_TO_RAD;
  left_knee_initial_pos_ = config.left_knee_initial_pos * DEG_TO_RAD;
  left_hip_initial_pos_ = config.left_hip_initial_pos * DEG_TO_RAD;

  right_ankle_initial_pos_ = config.right_ankle_initial_pos * DEG_TO_RAD;
  right_knee_initial_pos_ = config.right_knee_initial_pos * DEG_TO_RAD;
  right_hip_initial_pos_ = config.right_hip_initial_pos * DEG_TO_RAD;
}

void ImpulseController::callbackSubcriberJointState(
    sensor_msgs::JointState msg) {
  left_ankle_pos_ = msg.position.at(0);
  left_hip_pos_ = msg.position.at(1);
  left_knee_pos_ = msg.position.at(2);
  right_ankle_pos_ = msg.position.at(3);
  right_hip_pos_ = msg.position.at(4);
  right_knee_pos_ = msg.position.at(5);
}

bool ImpulseController::callbackServiceImpulse(
    legs_controllers::impulse::Request &req,
    legs_controllers::impulse::Response &res) {
  resetSimulation();

  return 1;
}
