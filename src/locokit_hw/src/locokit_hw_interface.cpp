#include "locokit_hw_ros/locokit_hw_interface.h"


LocokitHW::LocokitHW(ros::NodeHandle nh): nh_(nh), tcp_connected_(false)
{
    //Initialize parameters
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

        // Stop mottors just in case
        locokit_interface_->setActuatorStopped(locokitMotor::LEFT_HIP_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::LEFT_KNEE_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::LEFT_ANKLE_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_HIP_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_KNEE_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_ANKLE_ID);


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


bool LocokitHW::write(const motor* motors, int motornumber)
{
    //motornumber = _motor_number;
    motor* corrected_value[locokitMotor::NUMBER_MOTORS];

    // Initialization
    for (int motor = 0; motor < locokitMotor::NUMBER_MOTORS; ++motor)
       corrected_value[motor] = 0;

    std::cout << "Motors value:" << std::endl;
    std::cout << "   Motor " << locokitMotor::FRONT_LEFT_ID << ": "
             << std::to_string(motors[0]) << std::endl;
    std::cout << "   Motor " << locokitMotor::FRONT_RIGHT_ID << ": "
             << std::to_string(motors[1]) << std::endl;
    std::cout << "   Motor " << locokitMotor::REAR_LEFT_ID << ": "
             << std::to_string(motors[2]) << std::endl;
    std::cout << "   Motor " << locokitMotor::REAR_RIGHT_ID << ": "
             << std::to_string(motors[3]) << std::endl;
    std::cout << std::endl;

    *corrected_value[locokitMotor::FRONT_LEFT] = motors[locokitMotor::FRONT_LEFT] * 1.0;
    *corrected_value[locokitMotor::FRONT_RIGHT] = motors[locokitMotor::FRONT_RIGHT] * 1.0;
    *corrected_value[locokitMotor::REAR_LEFT] = motors[locokitMotor::REAR_LEFT] * 1.0;
    *corrected_value[locokitMotor::REAR_RIGHT] = motors[locokitMotor::REAR_RIGHT] * 1.0;


    // increase time counter
    _step_count++;
}














