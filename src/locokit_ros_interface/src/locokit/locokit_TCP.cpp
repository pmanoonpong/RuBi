#include "locokit_TCP.h"

Locokit_TCP::Locokit_TCP()
    : AbstractRobot("locokit_tcp", "now")
    , _connected(false)
{
    // Create the interface
    _locokit_interface = new LocoKitInterface(IP, PORT);

    // Establish connection
    if (_locokit_interface->establish_connection() == -1) {
        std::cout << "Error from LocoKitInterface: a connection couldn't be established..."
                  << std::endl;
        _connected = false;
    }
    else {
        std::cout << "Connected to robot!" << std::endl;
        _connected = true;
        // Init variables
        _step_count = 0; // global step counter
        _sensor_number = locokitSensor::NUMBER_SENSORS;
        _motor_number = locokitMotor::NUMBER_MOTORS;

        // Stop mottors just in case
        _locokit_interface->setActuatorStopped(locokitMotor::FRONT_LEFT_ID);
        _locokit_interface->setActuatorStopped(locokitMotor::FRONT_RIGHT_ID);
        _locokit_interface->setActuatorStopped(locokitMotor::REAR_LEFT_ID);
        _locokit_interface->setActuatorStopped(locokitMotor::REAR_RIGHT_ID);

        sleep(1);

        _locokit_interface->setActuatorPWM(300, locokitMotor::REAR_LEFT_ID);
        _locokit_interface->setActuatorPWM(300, locokitMotor::REAR_RIGHT_ID);

        usleep(500000);

        _locokit_interface->setActuatorPWM(300, locokitMotor::FRONT_LEFT_ID);
        _locokit_interface->setActuatorPWM(300, locokitMotor::FRONT_RIGHT_ID);
    }
}

Locokit_TCP::~Locokit_TCP()
{
    if (_connected) {
        // Stop mottors just in case
        _locokit_interface->setActuatorStopped(locokitMotor::FRONT_LEFT_ID);
        _locokit_interface->setActuatorStopped(locokitMotor::FRONT_RIGHT_ID);
        _locokit_interface->setActuatorStopped(locokitMotor::REAR_LEFT_ID);
        _locokit_interface->setActuatorStopped(locokitMotor::REAR_RIGHT_ID);
        _locokit_interface->terminate_connection_with_server();
    }
}

int Locokit_TCP::getSensors(sensor* sensors, int sensornumber)
{
    //sensornumber = _sensor_number;

    float measure;

    //DOUBT: Needs to be scaled to [-1, 1]
    _locokit_interface->getActuatorPosition(locokitMotor::FRONT_LEFT_ID, measure);
    sensors[locokitSensor::FRONT_LEFT_SPEED] = measure;
    _locokit_interface->getActuatorPosition(locokitMotor::FRONT_RIGHT_ID, measure);
    sensors[locokitSensor::FRONT_RIGHT_SPEED] = measure;
    _locokit_interface->getActuatorPosition(locokitMotor::REAR_LEFT_ID, measure);
    sensors[locokitSensor::REAR_LEFT_SPEED] = measure;
    _locokit_interface->getActuatorPosition(locokitMotor::REAR_RIGHT_ID, measure);
    sensors[locokitSensor::REAR_RIGHT_SPEED] = measure;

    std::cout << "Sensor value:" << std::endl;
    std::cout << "   Speed " << locokitMotor::FRONT_LEFT_ID << ": "
              << std::to_string(sensors[locokitSensor::FRONT_LEFT_SPEED]) << std::endl;
    std::cout << "   Speed " << locokitMotor::FRONT_RIGHT_ID << ": "
              << std::to_string(sensors[locokitSensor::FRONT_RIGHT_SPEED]) << std::endl;
    std::cout << "   Speed " << locokitMotor::REAR_LEFT_ID << ": "
              << std::to_string(sensors[locokitSensor::REAR_LEFT_SPEED]) << std::endl;
    std::cout << "   Speed " << locokitMotor::REAR_RIGHT_ID << ": "
              << std::to_string(sensors[locokitSensor::REAR_RIGHT_SPEED]) << std::endl;
    std::cout << std::endl;

    // Measure variable
    return this->_sensor_number;
}


void Locokit_TCP::setMotors(const motor* motors, int motornumber)
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

    /*
    _locokit_interface->setConstantSpeedInterpolatingFunction(
        locokitMotor::FRONT_LEFT_ID, (float)*corrected_value[locokitMotor::FRONT_LEFT], 0, 1);
    _locokit_interface->setConstantSpeedInterpolatingFunction(
        locokitMotor::FRONT_RIGHT_ID, (float)*corrected_value[locokitMotor::FRONT_RIGHT], 0, 0);
    _locokit_interface->setConstantSpeedInterpolatingFunction(
        locokitMotor::REAR_LEFT_ID, (float)*corrected_value[locokitMotor::REAR_LEFT], 90, 1);
    _locokit_interface->setConstantSpeedInterpolatingFunction(
        locokitMotor::REAR_RIGHT_ID, (float)*corrected_value[locokitMotor::REAR_RIGHT], 90, 0);
*/
    // increase time counter
    _step_count++;
}

bool Locokit_TCP::isConnected()
{
    return _connected;
}
