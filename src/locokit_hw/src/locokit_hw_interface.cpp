#include "locokit_hw_ros/locokit_hw_interface.h"


LocokitHW::LocokitHW(ros::NodeHandle nh): nh_(nh), tcp_connected_(false), sensors_(0), motors_(0)
{
    //Initialize parameters
    step_count_ = 0;
}


bool LocokitHW::configure()
{
    //Create the interface
    locokit_interface_ = new LocoKitInterface(IP, PORT);

    //Establish connection
    if (locokit_interface_->establish_connection() == -1) {
        std::cout << "Error from LocoKitInterface: a connection couldn't be established..."
                  << std::endl;
        tcp_connected_ = false;
        return false;
    }
    else {
        //If the connection successes
        std::cout << "Connected to robot!" << std::endl;
        tcp_connected_ = true;
        //Init variables
        step_count_ = 0; // global step counter
        sensor_number_ = locokitSensor::NUMBER_SENSORS;
        motor_number_ = locokitMotor::NUMBER_MOTORS;

        //Stop motors
        locokit_interface_->setActuatorStopped(locokitMotor::LEFT_HIP_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::LEFT_KNEE_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::LEFT_ANKLE_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_HIP_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_KNEE_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_ANKLE_ID);

        //TODO: take robot to initial configuration and set sensors to zero

        //Register hardware interface handlers:

        //Joint state handlers: read the state of a single joint
        hardware_interface::JointStateHandle state_joint("HIP_LEFT", &pos[locokitMotor::HIP_LEFT],
                                        &vel[locokitMotor::HIP_LEFT], &eff[locokitMotor::HIP_LEFT]);
        joint_state_interface.registerHandle(state_joint);

        //Joint command handlers: read and command a single joint
        //TODO

        sleep(1);
        return true;
    }
}


bool LocokitHW::start()
{
  if(!tcp_connected_) {
    ROS_ERROR("Could not connect to WiFi network.");
    return false;
    //TODO: exit program here
  }

  return true;
}


bool LocokitHW::read()
{
    //TODO: make it a for loop?
    float measure = 0.0;
    bool failed_to_read = false;

    //DOUBT: Needs to be scaled to [-1, 1]
    if(locokit_interface_->getActuatorPosition(locokitMotor::LEFT_HIP_ID, measure)!=-1){
        sensors_[locokitSensor::LEFT_HIP_SPEED] = measure;
    }
    else
        failed_to_read = true;

    if(locokit_interface_->getActuatorPosition(locokitMotor::LEFT_KNEE_ID, measure)!=-1){
        sensors_[locokitSensor::LEFT_KNEE_SPEED] = measure;
    }
    else
        failed_to_read = true;

    if(locokit_interface_->getActuatorPosition(locokitMotor::LEFT_ANKLE_ID, measure)!=-1){
        sensors_[locokitSensor::LEFT_ANKLE_SPEED] = measure;
    }
    else
        failed_to_read = true;

    if(locokit_interface_->getActuatorPosition(locokitMotor::RIGHT_HIP_ID, measure)!=-1){
        sensors_[locokitSensor::RIGHT_HIP_SPEED] = measure;
    }
    else
        failed_to_read = true;

    if(locokit_interface_->getActuatorPosition(locokitMotor::RIGHT_KNEE_ID, measure)!=-1){
        sensors_[locokitSensor::RIGHT_KNEE_SPEED] = measure;
    }
    else
        failed_to_read = true;

    if(locokit_interface_->getActuatorPosition(locokitMotor::RIGHT_ANKLE_ID, measure)!=-1){
        sensors_[locokitSensor::RIGHT_ANKLE_SPEED] = measure;
    }
    else
        failed_to_read = true;

    if(failed_to_read == false) return true;
    else
        return false;
}


void LocokitHW::write()
{
    //Create pointer array to be able to cast motor values to float
    motor* corrected_value[locokitMotor::NUMBER_MOTORS];

    for (int motor = 0; motor < locokitMotor::NUMBER_MOTORS; ++motor)
       corrected_value[motor] = 0;

    *corrected_value[locokitMotor::HIP_LEFT] = motors_[locokitMotor::HIP_LEFT] * 1.0;
    *corrected_value[locokitMotor::KNEE_LEFT] = motors_[locokitMotor::KNEE_LEFT] * 1.0;
    *corrected_value[locokitMotor::ANKLE_LEFT] = motors_[locokitMotor::ANKLE_LEFT] * 1.0;
    *corrected_value[locokitMotor::HIP_RIGHT] = motors_[locokitMotor::HIP_RIGHT] * 1.0;
    *corrected_value[locokitMotor::KNEE_RIGHT] = motors_[locokitMotor::KNEE_RIGHT] * 1.0;
    *corrected_value[locokitMotor::ANKLE_RIGHT] = motors_[locokitMotor::ANKLE_RIGHT] * 1.0;

    //Locokit Interface: set motor PWM
    locokit_interface_->setActuatorPWM(float(*corrected_value[locokitMotor::HIP_LEFT]), locokitMotor::LEFT_HIP_ID);
    locokit_interface_->setActuatorPWM(float(*corrected_value[locokitMotor::KNEE_LEFT]), locokitMotor::LEFT_KNEE_ID);
    locokit_interface_->setActuatorPWM(float(*corrected_value[locokitMotor::ANKLE_LEFT]), locokitMotor::LEFT_ANKLE_ID);
    locokit_interface_->setActuatorPWM(float(*corrected_value[locokitMotor::HIP_RIGHT]), locokitMotor::RIGHT_HIP_ID);
    locokit_interface_->setActuatorPWM(float(*corrected_value[locokitMotor::KNEE_RIGHT]), locokitMotor::RIGHT_KNEE_ID);
    locokit_interface_->setActuatorPWM(float(*corrected_value[locokitMotor::ANKLE_RIGHT]), locokitMotor::RIGHT_ANKLE_ID);


    // increase time counter
    step_count_++;
}


LocokitHW::~LocokitHW()
{
    if(tcp_connected_){
        // Stop motors
        locokit_interface_->setActuatorStopped(locokitMotor::LEFT_HIP_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::LEFT_KNEE_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::LEFT_ANKLE_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_HIP_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_KNEE_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_ANKLE_ID);
        //Close TCP connection with robot
        locokit_interface_->terminate_connection_with_server();
    }
}














