#include "locokit_hw_ros/locokit_hw_interface.h"


LocokitHW::LocokitHW(ros::NodeHandle nh): nh_(nh)
{
    //Initialize parameters
    tcp_connected_ = false;
    step_count_ = 0;

    for(unsigned int i=0; i<locokitMotor::NUMBER_MOTORS; i++){
        motors_[i] = 0.0;
    }
    for(unsigned int i=0; i<locokitSensor::NUMBER_SENSORS; i++){
        sensors_[i] = 0;
    }

    //Store names of the joints
    joint_names.insert(std::pair<int, std::string>(locokitMotor::HIP_LEFT, "left_hip"));
    joint_names.insert(std::pair<int, std::string>(locokitMotor::KNEE_LEFT, "left_knee"));
    joint_names.insert(std::pair<int, std::string>(locokitMotor::ANKLE_LEFT, "left_ankle"));
    joint_names.insert(std::pair<int, std::string>(locokitMotor::HIP_RIGHT, "right_hip"));
    //joint_names.insert(std::pair<int, std::string>(locokitMotor::KNEE_RIGHT, "right_knee"));
    //joint_names.insert(std::pair<int, std::string>(locokitMotor::ANKLE_RIGHT, "right_ankle"));

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
        //locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_KNEE_ID);
        //locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_ANKLE_ID);

        //TODO: take robot to initial configuration and set sensors to zero

        //Register hardware interface handlers:
        //Joint state handlers: read the state of a single joint
        typedef std::map<int, std::string>::iterator it_type;
        for(it_type iterator = joint_names.begin(); iterator != joint_names.end(); iterator++) {
            hardware_interface::JointStateHandle state_joint(iterator->second, &sensors_[iterator->first],
                                                        &vel[iterator->first], &eff[iterator->first]);
            joint_state_interface_.registerHandle(state_joint);
        }

        //Joint command handlers: read and command a single joint
        for(it_type iterator = joint_names.begin(); iterator != joint_names.end(); iterator++){
            hardware_interface::JointHandle command_joint(joint_state_interface_.getHandle(iterator->second),
                                                          &motors_[iterator->first]);
            joint_command_interface_.registerHandle(command_joint);
        }

        this->registerInterface(&joint_state_interface_);
        this->registerInterface(&joint_command_interface_);
        ROS_INFO("Registering HW interfaces");
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

    if(locokit_interface_->getActuatorPosition(locokitMotor::LEFT_HIP_ID, measure)==-1) failed_to_read = true;
    else
        sensors_[locokitSensor::LEFT_HIP_SPEED] = measure;

    if(locokit_interface_->getActuatorPosition(locokitMotor::LEFT_KNEE_ID, measure)==-1) failed_to_read = true;
    else
        sensors_[locokitSensor::LEFT_KNEE_SPEED] = measure;

    if(locokit_interface_->getActuatorPosition(locokitMotor::LEFT_ANKLE_ID, measure)==-1) failed_to_read = true;
    else
        sensors_[locokitSensor::LEFT_ANKLE_SPEED] = measure;

    if(locokit_interface_->getActuatorPosition(locokitMotor::RIGHT_HIP_ID, measure)==-1) failed_to_read = true;
    else
        sensors_[locokitSensor::RIGHT_HIP_SPEED] = measure;

    /*if(locokit_interface_->getActuatorPosition(locokitMotor::RIGHT_KNEE_ID, measure)==-1) failed_to_read = true;
    else
        sensors_[locokitSensor::RIGHT_KNEE_SPEED] = measure;

    if(locokit_interface_->getActuatorPosition(locokitMotor::RIGHT_ANKLE_ID, measure)==-1) failed_to_read = true;
    else
        sensors_[locokitSensor::RIGHT_ANKLE_SPEED] = measure;
    */
    if(failed_to_read == false) return true;
    else
        return false;
}


bool LocokitHW::write()
{
    bool failed_write = false;

    //TODO: map values from controller to PWM values

    //Locokit Interface: set motor PWM
    if(locokit_interface_->setActuatorPWM((float)motors_[locokitMotor::HIP_LEFT], locokitMotor::LEFT_HIP_ID) == -1) failed_write = true;
    std::cout<<motors_[locokitMotor::HIP_LEFT]<<std::endl;
    if(locokit_interface_->setActuatorPWM((float)motors_[locokitMotor::KNEE_LEFT], locokitMotor::LEFT_KNEE_ID) == -1) failed_write = true;
    std::cout<<motors_[locokitMotor::KNEE_LEFT]<<std::endl;
    if(locokit_interface_->setActuatorPWM((float)motors_[locokitMotor::ANKLE_LEFT], locokitMotor::LEFT_ANKLE_ID) == -1) failed_write = true;
    std::cout<<motors_[locokitMotor::ANKLE_LEFT]<<std::endl;
    if(locokit_interface_->setActuatorPWM((float)motors_[locokitMotor::HIP_RIGHT], locokitMotor::RIGHT_HIP_ID) == -1) failed_write = true;
    std::cout<<motors_[locokitMotor::HIP_RIGHT]<<std::endl;
    //if(locokit_interface_->setActuatorPWM(float(motors_[locokitMotor::KNEE_RIGHT]), locokitMotor::RIGHT_KNEE_ID) == -1) failed_write = true;
    //if(locokit_interface_->setActuatorPWM(float(motors_[locokitMotor::ANKLE_RIGHT]), locokitMotor::RIGHT_ANKLE_ID) == -1) failed_write = true;


    // increase time counter
    step_count_++;
    return failed_write;
}


LocokitHW::~LocokitHW()
{
    if(tcp_connected_){
        // Stop motors
        locokit_interface_->setActuatorStopped(locokitMotor::LEFT_HIP_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::LEFT_KNEE_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::LEFT_ANKLE_ID);
        locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_HIP_ID);
        //locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_KNEE_ID);
        //locokit_interface_->setActuatorStopped(locokitMotor::RIGHT_ANKLE_ID);
        //Close TCP connection with robot
        ROS_INFO("Terminate connection with server");
        locokit_interface_->terminate_connection_with_server();
    }
}














