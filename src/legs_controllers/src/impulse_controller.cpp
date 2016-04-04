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
                         "/legs/left_ankle_effort/command");
  nh_.param<std::string>("left_knee_effort_topic",
                         topic_effort_controller_left_knee_,
                         "/legs/left_knee_effort/command");
  nh_.param<std::string>("left_hip_effort_topic",
                         topic_effort_controller_left_hip_,
                         "/legs/left_hip_effort/command");
  nh_.param<std::string>("right_ankle_effort_topic",
                         topic_effort_controller_right_ankle_,
                         "/legs/right_ankle_effort/command");
  nh_.param<std::string>("right_knee_effort_topic",
                         topic_effort_controller_right_knee_,
                         "/legs/right_knee_effort/command");
  nh_.param<std::string>("right_hip_effort_topic",
                         topic_effort_controller_right_hip_,
                         "/legs/right_hip_effort/command");

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
  nh_.param<std::string>("gazebo_set_model_state_topic",
                         topic_gazebo_set_model_state_,
                         "/gazebo/set_model_state");
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

  nh_.getParam("left_ankle_initial_pos", left_ankle_initial_pos_);
  nh_.getParam("left_knee_initial_pos", left_knee_initial_pos_);
  nh_.getParam("left_hip_initial_pos", left_hip_initial_pos_);
  nh_.getParam("right_ankle_initial_pos", right_ankle_initial_pos_);
  nh_.getParam("right_knee_initial_pos", right_knee_initial_pos_);
  nh_.getParam("right_hip_initial_pos", right_hip_initial_pos_);

  nh_.getParam("hopping_ankle_pos", hopping_ankle_pos_);
  nh_.getParam("hopping_knee_pos", hopping_knee_pos_);
  nh_.getParam("hopping_hip_pos", hopping_hip_pos_);

  nh_.getParam("hopping_hip_pos", hopping_hip_pos_);
  nh_.getParam("hopping_hip_pos", hopping_hip_pos_);

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
  srv_client_gazebo_set_model_state_ =
      nh_.serviceClient<gazebo_msgs::SetModelState>(
          topic_gazebo_set_model_state_);

  srv_controller_manager_list_ =
      nh_.serviceClient<controller_manager_msgs::ListControllers>(
          topic_controller_manager_list_);
  srv_controller_manager_load_ =
      nh_.serviceClient<controller_manager_msgs::LoadController>(
          topic_controller_manager_load_);
  srv_controller_manager_switch_ =
      nh_.serviceClient<controller_manager_msgs::SwitchController>(
          topic_controller_manager_switch_);

  // Load all the controllers
  loadPositionAndEffortControllers();
}

void ImpulseController::step() {
  ros::spinOnce();
  update_rate_mutex_.lock();
  rate_->sleep();
  update_rate_mutex_.unlock();
}

void ImpulseController::resetSimulation() {
  // Resets the simulation
  std_srvs::Empty reset_simualtion_srv;
  srv_client_reset_simulation.call(reset_simualtion_srv);

  // Sets initial positions
  gazebo_msgs::SetModelConfiguration initial_configuration_msg;
  initial_configuration_msg.request.model_name = "legs";
  initial_configuration_msg.request.urdf_param_name = "robot_description";

  initial_configuration_msg.request.joint_names.push_back("left_hip");
  initial_configuration_msg.request.joint_positions.push_back(
      left_hip_initial_pos_ * DEG_TO_RAD);
  initial_configuration_msg.request.joint_names.push_back("left_knee");
  initial_configuration_msg.request.joint_positions.push_back(
      left_knee_initial_pos_ * DEG_TO_RAD);
  initial_configuration_msg.request.joint_names.push_back("left_ankle");
  initial_configuration_msg.request.joint_positions.push_back(
      left_ankle_initial_pos_ * DEG_TO_RAD);

  initial_configuration_msg.request.joint_names.push_back("right_hip");
  initial_configuration_msg.request.joint_positions.push_back(
      right_hip_initial_pos_ * DEG_TO_RAD);
  initial_configuration_msg.request.joint_names.push_back("right_knee");
  initial_configuration_msg.request.joint_positions.push_back(
      right_knee_initial_pos_ * DEG_TO_RAD);
  initial_configuration_msg.request.joint_names.push_back("right_ankle");
  initial_configuration_msg.request.joint_positions.push_back(
      right_ankle_initial_pos_ * DEG_TO_RAD);

  srv_client_gazebo_set_model_configuration_.call(initial_configuration_msg);

  // Set initial state
  gazebo_msgs::SetModelState initial_state_msg;

  double diff_height = 0.085;

  initial_state_msg.request.model_state.model_name = "legs";
  initial_state_msg.request.model_state.pose.position.z = -diff_height;

  srv_client_gazebo_set_model_state_.call(initial_state_msg);

  // Create another instance
  gazebo_msgs::SpawnModel spawn_model_msg;
  spawn_model_msg.request.model_name="legs2";
  spawn_model_msg.request.robot_namespace="legs2";
  nh_.getParam("/robot_description", spawn_model_msg.request.model_xml);
  spawn_model_msg.request.initial_pose.position.x = 1;
  ros::ServiceClient srv;
  srv =
      nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  srv.call(spawn_model_msg);

}

void ImpulseController::loadPositionAndEffortControllers() {
  // Prepares variables
  std::vector<std::string> controllers_to_load{
      "left_ankle_position",  "left_knee_position",  "left_hip_position",
      "right_ankle_position", "right_knee_position", "right_hip_position",
      "left_ankle_effort",    "left_knee_effort",    "left_hip_effort",
      "right_ankle_effort",   "right_knee_effort",   "right_hip_effort"};

  // Gets the controllers
  controller_manager_msgs::ListControllers list_controllers_msg;
  srv_controller_manager_list_.call(list_controllers_msg);

  // For all the moto in the right leg
  for (unsigned char i = 0; i < controllers_to_load.size(); ++i) {
    bool controller_loaded = false;
    // Check in the all the controller's list
    for (auto &controller : list_controllers_msg.response.controller) {
      // Finds out if the controller exists
      if (controller.name == controllers_to_load.at(i))
        controller_loaded = true;
    }

    if (!controller_loaded) {
      ROS_INFO_STREAM(controllers_to_load.at(i) << " is not loaded. Loading!");
      controller_manager_msgs::LoadController load_controller_msg;
      load_controller_msg.request.name = controllers_to_load.at(i);
      srv_controller_manager_load_.call(load_controller_msg);
    }
  }
}

void ImpulseController::setController(JOINT joint, CONTROLLER controller) {
  // Gets the controllers
  controller_manager_msgs::ListControllers list_controllers_msg;
  srv_controller_manager_list_.call(list_controllers_msg);

  bool switch_controller(true);
  // Check in the all the controller's list
  for (auto &controller_from_srv : list_controllers_msg.response.controller) {
    // If the controller is running
    if (controller_from_srv.name == (getName(joint) + getName(controller)) &&
        controller_from_srv.state == "running") {
      switch_controller = false;
    }
  }
  if (switch_controller) {
    /*ROS_INFO_STREAM("Switching " << getName(joint) << " to "
                                 << getName(controller)); */
    // Controller to stop
    CONTROLLER controller_to_stop;
    if (controller == CONTROLLER::EFFORT)
      controller_to_stop = CONTROLLER::POSITION;
    else
      controller_to_stop = CONTROLLER::EFFORT;

    // Stops the effort and starts the position controller
    controller_manager_msgs::SwitchController switch_controller_msg;
    switch_controller_msg.request.start_controllers.push_back(
        getName(joint) + getName(controller));
    switch_controller_msg.request.stop_controllers.push_back(
        getName(joint) + getName(controller_to_stop));
    switch_controller_msg.request.strictness = 1;
    srv_controller_manager_switch_.call(switch_controller_msg);
  }
}

std::string ImpulseController::getName(CONTROLLER controller) {
  switch (controller) {
    case (POSITION):
      return "position";
    case (EFFORT):
      return "effort";
    default:
      return "";
  }
}

std::string ImpulseController::getName(JOINT joint) {
  switch (joint) {
    case (LEFT_ANKLE):
      return "left_ankle_";
    case (LEFT_KNEE):
      return "left_knee_";
    case (LEFT_HIP):
      return "left_hip_";
    case (RIGHT_ANKLE):
      return "right_ankle_";
    case (RIGHT_KNEE):
      return "right_knee_";
    case (RIGHT_HIP):
      return "right_hip_";
    default:
      return "";
  }
}

void ImpulseController::hoppingPosition() {
  setController(JOINT::LEFT_ANKLE, CONTROLLER::EFFORT);
  setController(JOINT::LEFT_KNEE, CONTROLLER::EFFORT);
  setController(JOINT::LEFT_HIP, CONTROLLER::EFFORT);
  setController(JOINT::RIGHT_ANKLE, CONTROLLER::POSITION);
  setController(JOINT::RIGHT_KNEE, CONTROLLER::POSITION);
  setController(JOINT::RIGHT_HIP, CONTROLLER::POSITION);

  // Sends the hopping position
  std_msgs::Float64 motor_position_msg;
  motor_position_msg.data = hopping_ankle_pos_;
  pub_position_controller_right_ankle_.publish(motor_position_msg);
  motor_position_msg.data = hopping_knee_pos_;
  pub_position_controller_right_knee_.publish(motor_position_msg);
  motor_position_msg.data = hopping_hip_pos_;
  pub_position_controller_right_hip_.publish(motor_position_msg);
}

void ImpulseController::setEffortControllers() {
  setController(JOINT::LEFT_ANKLE, CONTROLLER::EFFORT);
  setController(JOINT::LEFT_KNEE, CONTROLLER::EFFORT);
  setController(JOINT::LEFT_HIP, CONTROLLER::EFFORT);
  setController(JOINT::RIGHT_ANKLE, CONTROLLER::EFFORT);
  setController(JOINT::RIGHT_KNEE, CONTROLLER::EFFORT);
  setController(JOINT::RIGHT_HIP, CONTROLLER::EFFORT);
}

void ImpulseController::setPositionControllers() {
  setController(JOINT::LEFT_ANKLE, CONTROLLER::POSITION);
  setController(JOINT::LEFT_KNEE, CONTROLLER::POSITION);
  setController(JOINT::LEFT_HIP, CONTROLLER::POSITION);
  setController(JOINT::RIGHT_ANKLE, CONTROLLER::POSITION);
  setController(JOINT::RIGHT_KNEE, CONTROLLER::POSITION);
  setController(JOINT::RIGHT_HIP, CONTROLLER::POSITION);
}

void ImpulseController::updateRate() {
  while (!killed) {
    if (ros::service::exists(topic_gazebo_physic_properties_,
                             false)) {  // Checks if gazebo exists
      gazebo_msgs::GetPhysicsProperties msg;
      srv_client_gazebo_physic_properties_.call(msg);
      if (msg.response.time_step != time_step_ ||
          msg.response.max_update_rate != real_time_factor_) {
        ROS_INFO_STREAM("Time changed! Real Time Factor: "
                        << msg.response.max_update_rate
                        << " Time Step: " << msg.response.time_step);
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

void ImpulseController::callbackDynamicParameters(
    legs_controller::impulse_controllerConfig &config, uint32_t level) {
  left_ankle_initial_pos_ = config.left_ankle_initial_pos * DEG_TO_RAD;
  left_knee_initial_pos_ = config.left_knee_initial_pos * DEG_TO_RAD;
  left_hip_initial_pos_ = config.left_hip_initial_pos * DEG_TO_RAD;

  right_ankle_initial_pos_ = config.right_ankle_initial_pos * DEG_TO_RAD;
  right_knee_initial_pos_ = config.right_knee_initial_pos * DEG_TO_RAD;
  right_hip_initial_pos_ = config.right_hip_initial_pos * DEG_TO_RAD;

  hopping_ankle_pos_ = config.hopping_ankle_pos * DEG_TO_RAD;
  hopping_knee_pos_ = config.hopping_knee_pos * DEG_TO_RAD;
  hopping_hip_pos_ = config.hopping_hip_pos * DEG_TO_RAD;
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

bool ImpulseController::callbackServiceImpulseOneLeg(
    legs_controllers::impulse_one_leg::Request &req,
    legs_controllers::impulse_one_leg::Response &res) {
  ROS_INFO("Impulse with One Leg");

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

  // Adds a 0 in the end to stop the motor
  ankle_trajectory.push_back(0);
  knee_trajectory.push_back(0);
  hip_trajectory.push_back(0);

  // Sends the commands
  for (unsigned int step = 0; step < ankle_trajectory.size(); ++step) {
    std_msgs::Float64 ankle_msg;
    ankle_msg.data = ankle_trajectory.at(step);
    std_msgs::Float64 knee_msg;
    knee_msg.data = knee_trajectory.at(step);
    std_msgs::Float64 hip_msg;
    hip_msg.data = hip_trajectory.at(step);

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
  ROS_INFO("Impulse with Two Legs");

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

  // Adds a 0 in the end to stop the motor
  ankle_trajectory.push_back(0);
  knee_trajectory.push_back(0);
  hip_trajectory.push_back(0);

  // Sends the commands
  for (unsigned int step = 0; step < ankle_trajectory.size(); ++step) {
    std_msgs::Float64 ankle_msg;
    ankle_msg.data = ankle_trajectory.at(step);
    std_msgs::Float64 knee_msg;
    knee_msg.data = knee_trajectory.at(step);
    std_msgs::Float64 hip_msg;
    hip_msg.data = hip_trajectory.at(step);

    pub_effort_controller_left_ankle_.publish(ankle_msg);
    pub_effort_controller_right_ankle_.publish(ankle_msg);

    pub_effort_controller_left_knee_.publish(knee_msg);
    pub_effort_controller_right_knee_.publish(knee_msg);

    pub_effort_controller_left_hip_.publish(hip_msg);
    pub_effort_controller_right_hip_.publish(hip_msg);

    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(
        time_step / (time_step_ * real_time_factor_) * 1000));
  }

  return 1;
}
