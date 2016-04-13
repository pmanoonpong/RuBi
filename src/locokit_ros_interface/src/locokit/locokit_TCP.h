/**
 * @author Jorge Rodriguez 16 Jan 2016
 * jorod14@student.sdu.dk
 */
#ifndef LOCOKIT_TCP_H
#define LOCOKIT_TCP_H

#include <iostream>

#include <selforg/abstractrobot.h>
#include <selforg/matrix.h>

#include "robot_configuration.h"

#include "../../include/locokit/LocoKitInterface.h"

/**
 * This class communicates with Locokit Robot
 * REMEMBER to make congruent the configuration file
 * of this program (client side) and the one running in the
 * LocoKit (servers side)
 */
class Locokit_TCP : public AbstractRobot {
public:
    /*
     * Default constructor
     */
    Locokit_TCP();

    /*
     * Default destructor
     */
    ~Locokit_TCP();

    // robot interface
    /** returns actual sensorvalues
     * @param sensors sensors scaled to [-1,1]
     * @param sensornumber length of the sensor array
     * @return number of actually written sensors
     */
    virtual int getSensors(sensor* sensors, int _sensor_number);

    /** sets actual motorcommands
     * @param motors motors scaled to [-1,1]
     * @param motornumber length of the motor array
     */
    virtual void setMotors(const motor* motors, int _motor_number);

    /**
     * @brief getSensorNumber returns number of sensors
     */
    virtual int getSensorNumber() { return _sensor_number; }

    /**
     * @brief getMotorNumber returns number of motors
     */
    virtual int getMotorNumber() { return _motor_number; }

    /* the following are not used here, you can ignore them but keep them*/
    virtual Position getPosition() const { return Position(0, 0, 0); }
    virtual Position getSpeed() const { return Position(0, 0, 0); }
    virtual Position getAngularSpeed() const { return Position(0, 0, 0); }
    virtual matrix::Matrix getOrientation() const
    {
        matrix::Matrix m(3, 3);
        m.toId();
        return m;
    }

    bool isConnected();

private:
    LocoKitInterface* _locokit_interface;
    int _motor_number;
    int _sensor_number;
    int _step_count;
    bool _connected;
};
#endif
