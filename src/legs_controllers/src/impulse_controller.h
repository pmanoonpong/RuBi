#ifndef IMPULSE_CONTROLLER_H
#define IMPULSE_CONTROLLER_H

// System
#include <signal.h>

// STD
#include <vector>
#include <cmath>
#include <iostream>
#include <string>
#include <mutex>
#include <thread>

// ROS
#include "ros/ros.h"

#include "dynamic_reconfigure/server.h"

#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/JointState.h"

#include "gazebo_msgs/GetPhysicsProperties.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/SpawnModel.h"

#include "legs_controllers/impulse_one_leg.h"
#include "legs_controllers/impulse_two_legs.h"
#include "legs_controllers/impulse_controllerConfig.h"

#include "controller_manager_msgs/ListControllers.h"
#include "controller_manager_msgs/LoadController.h"
#include "controller_manager_msgs/SwitchController.h"

#define DEG_TO_RAD M_PI / 180
#define RAD_TO_DEG 180 / M_PI

// Defined by the protocol of the robot
enum JOINT {
  LEFT_ANKLE,
  LEFT_KNEE,
  LEFT_HIP,
  RIGHT_ANKLE,
  RIGHT_KNEE,
  RIGHT_HIP
};
enum CONTROLLER { EFFORT, POSITION };

class ImpulseController {
 public:
  ImpulseController();

  /**
   * @brief updateRate Updates the frequency at which the controller is updated.
   * This parameter is got from a default parameter from the launch file and, in
   * the case of being used with gazebo, from the simulator perse.
   */
  void updateRate();

  /**
   * @brief step
   */
  void step();

  /**
   * @brief restart The simulation and puts the robot in the inital position
   */
  void resetSimulation();

  /**
   * @brief hoppingPosition Folds the right leg to hopping position. This is
   * used for hop with the other leg. Made by changing the controllers of the
   * folded leg to position controllers.
   */
  void hoppingPosition();

  /**
   * @brief loadPositionAndEffortControllers Loads the position and the effort
   * controllers of the joints in case they are not.
   */
  void loadPositionAndEffortControllers();

  /**
   * @brief setController Set a controller to a joint. If it has already one,
   * switch it.
   */
  void setController(JOINT joint, CONTROLLER controller);

  /**
   * @brief getName Returns the name of the joint as expressed in the ROS param
   */
  std::string getName(JOINT joint);
  /**
   * @brief getName Returns the name of the controller as expressed in the ROS
   * param
   */
  std::string getName(CONTROLLER controller);

  /**
   * @brief setEffortControllers Set all the joint controllers to effort
   */
  void setEffortControllers();

  /**
   * @brief setPositionControllers Set all the joint controllers to position
   */
  void setPositionControllers();

  // Callbacks
  /**
   * @brief callbackSubcriberJointState
   * @param msg
   */
  void callbackSubcriberJointState(sensor_msgs::JointState msg);

  /**
   * @brief callbackDynamicParameters
   * @param config
   * @param level
   */
  void callbackDynamicParameters(
      legs_controller::impulse_controllerConfig& config, uint32_t level);

  /**
   * @brief callbackServiceImpulseOneLeg
   * @param req
   * @param res
   * @return
   */
  bool callbackServiceImpulseOneLeg(
      legs_controllers::impulse_one_leg::Request& req,
      legs_controllers::impulse_one_leg::Response& res);

  /**
   * @brief callbackServiceImpulseTwoLegs
   * @param req
   * @param res
   * @return
   */
  bool callbackServiceImpulseTwoLegs(
      legs_controllers::impulse_two_legs::Request& req,
      legs_controllers::impulse_two_legs::Response& res);

 private:
  // ROS
  ros::NodeHandle nh_;
  ros::Publisher pub_effort_controller_left_ankle_,
      pub_effort_controller_left_knee_, pub_effort_controller_left_hip_,
      pub_effort_controller_right_ankle_, pub_effort_controller_right_knee_,
      pub_effort_controller_right_hip_, pub_position_controller_left_ankle_,
      pub_position_controller_left_knee_, pub_position_controller_left_hip_,
      pub_position_controller_right_ankle_, pub_position_controller_right_knee_,
      pub_position_controller_right_hip_;

  ros::Subscriber sub_joint_states_;

  ros::ServiceClient srv_client_gazebo_physic_properties_,
      srv_client_gazebo_set_model_configuration_,
      srv_client_gazebo_set_model_state_, srv_client_reset_simulation,
      srv_controller_manager_list_, srv_controller_manager_load_,
      srv_controller_manager_switch_;

  ros::ServiceServer srv_server_impulse_one_leg_, srv_server_impulse_two_legs_;

  std::string topic_effort_controller_left_ankle_,
      topic_effort_controller_left_knee_, topic_effort_controller_left_hip_,
      topic_effort_controller_right_ankle_, topic_effort_controller_right_knee_,
      topic_effort_controller_right_hip_, topic_position_controller_left_ankle_,
      topic_position_controller_left_knee_, topic_position_controller_left_hip_,
      topic_position_controller_right_ankle_,
      topic_position_controller_right_knee_,
      topic_position_controller_right_hip_, topic_joint_states_,
      topic_gazebo_physic_properties_, topic_gazebo_set_model_configuration_,
      topic_gazebo_set_model_state_, topic_gazebo_reset_simulation_,
      topic_impulse_one_leg_, topic_impulse_two_legs_,
      topic_controller_manager_list_, topic_controller_manager_load_,
      topic_controller_manager_switch_;

  // Step time
  double time_step_;
  double real_time_factor_;
  double update_rate_;
  std::shared_ptr<ros::Rate> rate_;
  std::recursive_mutex update_rate_mutex_;
  std::thread update_rate_thread_;

  // Dynamic reconfigure
  typedef dynamic_reconfigure::Server<legs_controller::impulse_controllerConfig>
      DynamicReconfigServer;
  std::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
  DynamicReconfigServer::CallbackType param_reconfig_callback_;
  boost::recursive_mutex param_reconfig_mutex_;  // Must be boost

  // Controller: common
  double left_hip_pos_, left_knee_pos_, left_ankle_pos_, right_hip_pos_,
      right_knee_pos_, right_ankle_pos_;

  double left_hip_initial_pos_, left_knee_initial_pos_, left_ankle_initial_pos_,
      right_hip_initial_pos_, right_knee_initial_pos_, right_ankle_initial_pos_,
      hopping_ankle_pos_, hopping_knee_pos_, hopping_hip_pos_;
};

#endif  // IMPULSE_CONTROLLER_H
