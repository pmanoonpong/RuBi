#include "locokit_hw_ros/locokit_hw_interface.h"


LocokitHW::LocokitHW(ros::NodeHandle nh): nh_(nh), tcp_connected_(false)
{
    //Initialize parameters
    step_count_ = 0;
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


bool LocokitHW::configure()
{
    //Create the interface
    locokit_interface_ = new LocoKitInterface(IP, PORT);

    // Establish connection
    if (locokit_interface_->establish_connection() == -1) {
        std::cout << "Error from LocoKitInterface: a connection couldn't be established..."
                  << std::endl;
        tcp_connected_ = false;
    }
    else {
        std::cout << "Connected to robot!" << std::endl;
        tcp_connected_ = true;
        // Init variables
        step_count_ = 0; // global step counter
        sensor_number_ = locokitSensor::NUMBER_SENSORS;
        motor_number_ = locokitMotor::NUMBER_MOTORS;

        // Stop motors
        locokit_interface_->setActuatorStopped(locokitMotor::LEFT_HIP_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::LEFT_KNEE_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::LEFT_ANKLE_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_HIP_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_KNEE_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_ANKLE_ID);

        //TODO: take robot to initial configuration and set sensors to zero

        sleep(1);
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


bool LocokitHW::read(sensor* sensors, int sensors_number)
{
    float measure;

    //DOUBT: Needs to be scaled to [-1, 1]
    locokit_interface_->getActuatorPosition(locokitMotor::LEFT_HIP_ID, measure);
    sensors[locokitSensor::LEFT_HIP_SPEED] = measure;
    locokit_interface_->getActuatorPosition(locokitMotor::LEFT_KNEE_ID, measure);
    sensors[locokitSensor::LEFT_KNEE_SPEED] = measure;
    locokit_interface_->getActuatorPosition(locokitMotor::LEFT_ANKLE_ID, measure);
    sensors[locokitSensor::LEFT_ANKLE_SPEED] = measure;
    locokit_interface_->getActuatorPosition(locokitMotor::RIGHT_HIP_ID, measure);
    sensors[locokitSensor::RIGHT_HIP_SPEED] = measure;
    locokit_interface_->getActuatorPosition(locokitMotor::RIGHT_KNEE_ID, measure);
    sensors[locokitSensor::RIGHT_KNEE_SPEED] = measure;
    locokit_interface_->getActuatorPosition(locokitMotor::RIGHT_ANKLE_ID, measure);
    sensors[locokitSensor::RIGHT_ANKLE_SPEED] = measure;

    // Measure variable
    return this->sensor_number_;
}


void LocokitHW::write(const motor* motors, int motornumber)
{
    //Create pointer array to be able to cast motor values to float
    motor* corrected_value[locokitMotor::NUMBER_MOTORS];

    for (int motor = 0; motor < locokitMotor::NUMBER_MOTORS; ++motor)
       corrected_value[motor] = 0;

    *corrected_value[locokitMotor::HIP_LEFT] = motors[locokitMotor::HIP_LEFT] * 1.0;
    *corrected_value[locokitMotor::KNEE_LEFT] = motors[locokitMotor::KNEE_LEFT] * 1.0;
    *corrected_value[locokitMotor::ANKLE_LEFT] = motors[locokitMotor::ANKLE_LEFT] * 1.0;
    *corrected_value[locokitMotor::HIP_RIGHT] = motors[locokitMotor::HIP_RIGHT] * 1.0;
    *corrected_value[locokitMotor::KNEE_RIGHT] = motors[locokitMotor::KNEE_RIGHT] * 1.0;
    *corrected_value[locokitMotor::ANKLE_RIGHT] = motors[locokitMotor::ANKLE_RIGHT] * 1.0;

    //Locokit Interface: set motor PWM
    locokit_interface_->setActuatorPWM((float)*corrected_value[locokitMotor::HIP_LEFT], LEFT_HIP_ID);
    locokit_interface_->setActuatorPWM((float)*corrected_value[locokitMotor::KNEE_LEFT], LEFT_KNEE_ID);
    locokit_interface_->setActuatorPWM((float)*corrected_value[locokitMotor::ANKLE_LEFT], LEFT_ANKLE_ID);
    locokit_interface_->setActuatorPWM((float)*corrected_value[locokitMotor::HIP_RIGHT], RIGHT_HIP_ID);
    locokit_interface_->setActuatorPWM((float)*corrected_value[locokitMotor::KNEE_RIGHT], RIGHT_KNEE_ID);
    locokit_interface_->setActuatorPWM((float)*corrected_value[locokitMotor::ANKLE_RIGHT], RIGHT_ANKLE_ID);


    // increase time counter
    step_count_++;
}














