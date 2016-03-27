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

  nh_.param<std::string>("left_ankle_effort_topic",
                         topic_effort_controller_left_ankle_,
                         "/legs/left_ankle_effort");
  nh_.param<std::string>("left_knee_effort_topic",
                         topic_effort_controller_left_knee_,
                         "/legs/left_knee_effort");
  nh_.param<std::string>("left_hip_effort_topic",
                         topic_effort_controller_left_hip_,
                         "/legs/left_hip_effort");
  nh_.param<std::string>("right_ankle_effort_topic",
                         topic_effort_controller_right_ankle_,
                         "/legs/right_ankle_effort");
  nh_.param<std::string>("right_knee_effort_topic",
                         topic_effort_controller_right_knee_,
                         "/legs/right_knee_effort");
  nh_.param<std::string>("right_hip_effort_topic",
                         topic_effort_controller_right_hip_,
                         "/legs/right_hip_effort");

  nh_.param<std::string>("left_ankle_position_topic",
                         topic_position_controller_left_ankle_,
                         "/legs/left_ankle_position/command");
  nh_.param<std::string>("left_knee_position_topic",
                         topic_position_controller_left_knee_,
                         "/legs/left_knee_position/command");
  nh_.param<std::string>("left_hip_position_topic",
                         topic_position_controller_left_hip_,
                         "/legs/left_hip_position/command");
  nh_.param<std::string>("right_ankle_position_topic",
                         topic_position_controller_right_ankle_,
                         "/legs/right_ankle_position/command");
  nh_.param<std::string>("right_knee_position_topic",
                         topic_position_controller_right_knee_,
                         "/legs/right_knee_position/command");
  nh_.param<std::string>("right_hip_position_topic",
                         topic_position_controller_right_hip_,
                         "/legs/right_hip_position/command");

  nh_.param<std::string>("gazebo_physic_properties_topic",
                         topic_gazebo_physic_properties_,
                         "/gazebo/get_physics_properties");
  nh_.param<std::string>("gazebo_set_model_configuration_topic",
                         topic_gazebo_set_model_configuration_,
                         "/gazebo/set_model_configuration");
  nh_.param<std::string>("gazebo_reset_simulation_topic",
                         topic_gazebo_reset_simulation_,
                         "/gazebo/reset_simulation");

  nh_.param<std::string>("controller_manager_list_topic",
                         topic_controller_manager_list_,
                         "/legs/controller_manager/list_controllers");
  nh_.param<std::string>("controller_manager_load_topic",
                         topic_controller_manager_load_,
                         "/legs/controller_manager/load_controller");
  nh_.param<std::string>("controller_manager_switch_topic",
                         topic_controller_manager_switch_,
                         "/legs/controller_manager/switch_controller");

  nh_.param<std::string>("impulse_one_leg_topic", topic_impulse_one_leg_,
                         "/legs/impulse_one_leg");
  nh_.param<std::string>("impulse_two_legs_topic", topic_impulse_two_legs_,
                         "/legs/impulse_two_legs");

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
  pub_effort_controller_left_ankle_ =
      nh_.advertise<std_msgs::Float64>(topic_effort_controller_left_ankle_, 1);
  pub_effort_controller_left_knee_ =
      nh_.advertise<std_msgs::Float64>(topic_effort_controller_left_knee_, 1);
  pub_effort_controller_left_hip_ =
      nh_.advertise<std_msgs::Float64>(topic_effort_controller_left_hip_, 1);
  pub_effort_controller_right_ankle_ =
      nh_.advertise<std_msgs::Float64>(topic_effort_controller_right_ankle_, 1);
  pub_effort_controller_right_knee_ =
      nh_.advertise<std_msgs::Float64>(topic_effort_controller_right_knee_, 1);
  pub_effort_controller_right_hip_ =
      nh_.advertise<std_msgs::Float64>(topic_effort_controller_right_hip_, 1);

  pub_position_controller_left_ankle_ = nh_.advertise<std_msgs::Float64>(
      topic_position_controller_left_ankle_, 1);
  pub_position_controller_left_knee_ =
      nh_.advertise<std_msgs::Float64>(topic_position_controller_left_knee_, 1);
  pub_position_controller_left_hip_ =
      nh_.advertise<std_msgs::Float64>(topic_position_controller_left_hip_, 1);
  pub_position_controller_right_ankle_ = nh_.advertise<std_msgs::Float64>(
      topic_position_controller_right_ankle_, 1);
  pub_position_controller_right_knee_ = nh_.advertise<std_msgs::Float64>(
      topic_position_controller_right_knee_, 1);
  pub_position_controller_right_hip_ =
      nh_.advertise<std_msgs::Float64>(topic_position_controller_right_hip_, 1);

  // Services
  srv_server_impulse_one_leg_ = nh_.advertiseService(
      topic_impulse_one_leg_, &ImpulseController::callbackServiceImpulseOneLeg,
      this);
  srv_server_impulse_two_legs_ = nh_.advertiseService(
      topic_impulse_two_legs_,
      &ImpulseController::callbackServiceImpulseTwoLegs, this);

  srv_client_reset_simulation =
      nh_.serviceClient<std_srvs::Empty>(topic_gazebo_reset_simulation_);
  srv_client_gazebo_physic_properties_ =
      nh_.serviceClient<gazebo_msgs::GetPhysicsProperties>(
          topic_gazebo_physic_properties_);
  srv_client_gazebo_set_model_configuration_ =
      nh_.serviceClient<gazebo_msgs::SetModelConfiguration>(
          topic_gazebo_set_model_configuration_);

  srv_controller_manager_list_ =
      nh_.serviceClient<controller_manager_msgs::ListControllers>(
          topic_controller_manager_list_);
  srv_controller_manager_load_ =
      nh_.serviceClient<controller_manager_msgs::LoadController>(
          topic_controller_manager_load_);
  srv_controller_manager_switch_ =
      nh_.serviceClient<controller_manager_msgs::SwitchController>(
          topic_controller_manager_switch_);
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

void ImpulseController::hoppingPosition() {
  // Sets all the controllers to effort. Just in case the left leg is not
  // in effort mode.
  setEffortControllers();

  // Prepares variables for the right leg
  std::vector<std::string> right_position_controllers{
      "right_ankle_position", "right_knee_position", "right_hip_position"};
  std::vector<std::string> right_effort_controllers{
      "right_ankle_effort", "right_knee_effort", "right_hip_effort"};
  std::vector<double> right_hopping_positions{0 * DEG_TO_RAD, -90 * DEG_TO_RAD,
                                              40 * DEG_TO_RAD};
  std::vector<ros::Publisher *> publishers{
      &pub_position_controller_right_ankle_,
      &pub_position_controller_right_knee_,
      &pub_position_controller_right_hip_};

  // Check if the right leg controllers are positions.
  controller_manager_msgs::ListControllers list_controllers_msg;
  srv_controller_manager_list_.call(list_controllers_msg);

  // For all the motors in the right leg
  for (unsigned char right_leg_controller = 0;
       right_leg_controller < right_position_controllers.size();
       ++right_leg_controller) {
    bool controller_loaded(false);
    // Check in the all the controller's list
    for (auto &controller : list_controllers_msg.response.controller) {
      controller_loaded = true;
      // Find the position controller for that joint exists
      if (controller.name ==
          right_position_controllers.at(right_leg_controller)) {
        // If it does, checks if it is running
        if (controller.state != "running") {
          ROS_INFO_STREAM(right_position_controllers.at(right_leg_controller)
                          << " is loaded but not running. Starting!");
          // Stops the effort and starts the position controller
          controller_manager_msgs::SwitchController switch_controller_msg;
          switch_controller_msg.request.start_controllers.push_back(
              right_position_controllers.at(right_leg_controller));
          switch_controller_msg.request.stop_controllers.push_back(
              right_effort_controllers.at(right_leg_controller));
          switch_controller_msg.request.strictness = 1;
          srv_controller_manager_switch_.call(switch_controller_msg);
        }
      }
    }
    if (controller_loaded) {
      ROS_INFO_STREAM(right_position_controllers.at(right_leg_controller)
                      << " is not loaded. Spawning!");
      controller_manager_msgs::LoadController load_controller_msg;
      load_controller_msg.request.name =
          right_position_controllers.at(right_leg_controller);
      srv_controller_manager_load_.call(load_controller_msg);

      // Stops the effort and starts the position controller
      controller_manager_msgs::SwitchController switch_controller_msg;
      switch_controller_msg.request.start_controllers.push_back(
          right_position_controllers.at(right_leg_controller));
      switch_controller_msg.request.stop_controllers.push_back(
          right_effort_controllers.at(right_leg_controller));
      srv_controller_manager_switch_.call(switch_controller_msg);
    }

    // Sends the hopping position
    std_msgs::Float64 motor_position_msg;
    motor_position_msg.data = right_hopping_positions.at(right_leg_controller);
    publishers.at(right_leg_controller)->publish(motor_position_msg);
  }
}

void ImpulseController::setEffortControllers() {
  // Prepares variables
  std::vector<std::string> position_controllers{
      "left_ankle_position",  "left_knee_position",  "left_hip_position",
      "right_ankle_position", "right_knee_position", "right_hip_position"};
  std::vector<std::string> effort_controllers{
      "left_ankle_effort",  "left_knee_effort",  "left_hip_effort",
      "right_ankle_effort", "right_knee_effort", "right_hip_effort"};

  // Check if the right leg controllers are positions.
  controller_manager_msgs::ListControllers list_controllers_msg;
  srv_controller_manager_list_.call(list_controllers_msg);

  // For all the motors in the right leg
  for (unsigned char leg_controller = 0;
       leg_controller < effort_controllers.size(); ++leg_controller) {
    bool controller_loaded = false;
    // Check in the all the controller's list
    for (auto &controller : list_controllers_msg.response.controller) {
      // Finds out if the controller exists
      if (controller.name == effort_controllers.at(leg_controller)) {
        controller_loaded = true;
        // If it does, checks if it is running
        if (controller.state != "running") {
          ROS_INFO_STREAM(effort_controllers.at(leg_controller)
                          << " is loaded but not running. Starting!");
          // Stops the position and starts the effort controller
          controller_manager_msgs::SwitchController switch_controller_msg;
          switch_controller_msg.request.start_controllers.push_back(
              effort_controllers.at(leg_controller));
          switch_controller_msg.request.stop_controllers.push_back(
              position_controllers.at(leg_controller));
          switch_controller_msg.request.strictness = 1;
          srv_controller_manager_switch_.call(switch_controller_msg);
        }
      }
    }
    if (!controller_loaded) {
      ROS_INFO_STREAM(effort_controllers.at(leg_controller)
                      << " is not loaded. Spawning!");
      controller_manager_msgs::LoadController load_controller_msg;
      load_controller_msg.request.name =
          position_controllers.at(leg_controller);
      srv_controller_manager_load_.call(load_controller_msg);

      // Stops the effort and starts the position controller
      controller_manager_msgs::SwitchController switch_controller_msg;
      switch_controller_msg.request.start_controllers.push_back(
          effort_controllers.at(leg_controller));
      switch_controller_msg.request.stop_controllers.push_back(
          position_controllers.at(leg_controller));
      srv_controller_manager_switch_.call(switch_controller_msg);
    }
  }
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
  left_ankle_pos_ = msg.position.at(MOTORS::LEFT_ANKLE);
  left_hip_pos_ = msg.position.at(MOTORS::LEFT_KNEE);
  left_knee_pos_ = msg.position.at(MOTORS::LEFT_HIP);
  right_ankle_pos_ = msg.position.at(MOTORS::RIGHT_ANKLE);
  right_hip_pos_ = msg.position.at(MOTORS::RIGHT_KNEE);
  right_knee_pos_ = msg.position.at(MOTORS::RIGHT_HIP);
}

bool ImpulseController::callbackServiceImpulseOneLeg(
    legs_controllers::impulse_one_leg::Request &req,
    legs_controllers::impulse_one_leg::Response &res) {
  // Resets the simulation
  resetSimulation();

  // Put the right leg into hopping position
  hoppingPosition();

  // Creates the trajectory of the motors
  std::vector<double> ankle_trajectory;
  std::vector<double> knee_trajectory;
  std::vector<double> hip_trajectory;

  // Gets the last time step
  double time_step;
  update_rate_mutex_.lock();
  time_step = time_step_;
  update_rate_mutex_.unlock();

  // Adds the acceleration phase
  double acceleration_time = 0.1 * req.impulse_time;
  double acceleration_steps = acceleration_time / time_step;

  for (double step = 0; step < acceleration_steps; ++step) {
    ankle_trajectory.push_back(req.torque_ankle * step);
    knee_trajectory.push_back(req.torque_knee * step);
    hip_trajectory.push_back(req.torque_hip * step);
  }

  // Adds the impulse phase
  double impulse_steps = req.impulse_time / time_step;

  for (double step = 0; step < impulse_steps; ++step) {
    ankle_trajectory.push_back(req.torque_ankle);
    knee_trajectory.push_back(req.torque_knee);
    hip_trajectory.push_back(req.torque_hip);
  }

  // Sends the commands
  for (unsigned int step = 0; step < ankle_trajectory.size(); ++step) {
    std_msgs::Float64 ankle_msg;
    ankle_msg.data = ankle_trajectory.at(step);
    std_msgs::Float64 knee_msg;
    knee_msg.data = ankle_trajectory.at(step);
    std_msgs::Float64 hip_msg;
    hip_msg.data = ankle_trajectory.at(step);

    pub_effort_controller_left_ankle_.publish(ankle_msg);
    pub_effort_controller_left_knee_.publish(knee_msg);
    pub_effort_controller_left_hip_.publish(hip_msg);

    std::this_thread::sleep_for(
        std::chrono::duration<double, std::milli>(time_step * 1000));
  }

  return 1;
}

bool ImpulseController::callbackServiceImpulseTwoLegs(
    legs_controllers::impulse_two_legs::Request &req,
    legs_controllers::impulse_two_legs::Response &res) {
  // Resets the simulation
  resetSimulation();

  // Sets all the controllers to effort
  setEffortControllers();

  // Creates the trajectory of the motors
  std::vector<double> ankle_trajectory;
  std::vector<double> knee_trajectory;
  std::vector<double> hip_trajectory;

  // Gets the last time step
  double time_step;
  update_rate_mutex_.lock();
  time_step = 0.001/*time_step_*/;
  update_rate_mutex_.unlock();

  // Adds the acceleration phase
  double acceleration_time = 0.1 * req.impulse_time;
  double acceleration_steps = acceleration_time / time_step;

  for (double step = 0; step < acceleration_steps; ++step) {
    ankle_trajectory.push_back(req.torque_ankle * step);
    knee_trajectory.push_back(req.torque_knee * step);
    hip_trajectory.push_back(req.torque_hip * step);
  }

  // Adds the impulse phase
  double impulse_steps = req.impulse_time / time_step;

  for (double step = 0; step < impulse_steps; ++step) {
    ankle_trajectory.push_back(req.torque_ankle);
    knee_trajectory.push_back(req.torque_knee);
    hip_trajectory.push_back(req.torque_hip);
  }

  // Sends the commands
  for (unsigned int step = 0; step < ankle_trajectory.size(); ++step) {
    std_msgs::Float64 ankle_msg;
    ankle_msg.data = ankle_trajectory.at(step);
    std_msgs::Float64 knee_msg;
    knee_msg.data = ankle_trajectory.at(step);
    std_msgs::Float64 hip_msg;
    hip_msg.data = ankle_trajectory.at(step);

    pub_effort_controller_left_ankle_.publish(ankle_msg);
    pub_effort_controller_right_ankle_.publish(ankle_msg);

    pub_effort_controller_left_knee_.publish(knee_msg);
    pub_effort_controller_right_knee_.publish(knee_msg);

    pub_effort_controller_left_hip_.publish(hip_msg);
    pub_effort_controller_right_hip_.publish(hip_msg);

    std::this_thread::sleep_for(
        std::chrono::duration<double, std::milli>(time_step * 1000));
  }

  return 1;
}
