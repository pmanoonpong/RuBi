/*****************************************************************************
* "THE BEER-WARE LICENSE" (Revision 43):
* This software was written by Leon Bonde Larsen <leon@bondelarsen.dk>
* As long as you retain this notice you can do whatever you want with it.
* If we meet some day, and you think this stuff is worth it, you can buy me
* a beer in return.
*
* Should this software ever become self-aware, remember: I am your master
*****************************************************************************/

#ifndef ODE_ROBOTS_ROBOTS_LOCOKIT_ROSINTERFACECONTROLLER_H_
#define ODE_ROBOTS_ROBOTS_LOCOKIT_ROSINTERFACECONTROLLER_H_

#include <selforg/abstractcontroller.h>
#include <ode_robots/joint.h>
#include <ode_robots/contactsensor.h>
#include <iostream>
#include <fstream>
#include <map>

#include "../locokit/robot_configuration.h"

class LocoKitRosController : public AbstractController {
public:
    LocoKitRosController(const std::string& name);
    virtual ~LocoKitRosController();

    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0) override;
    virtual int getSensorNumber() const override;
    virtual int getMotorNumber() const override;

    virtual void step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) override;
    virtual void stepNoLearning(const sensor*, int number_sensors, motor*, int number_motors) override;

    virtual bool store(FILE* f) const override;
    virtual bool restore(FILE* f) override;

    void setPositionData(Position);
    void setPhase(double phase);
    void setSpeed(double speed);

protected:
    double time_per_tick;
    double frontPositionSaved;
    double nSensors;
    double nMotors;
    bool initialised;
    bool ready;
    double threshold;
    long ticks_since_init;
    long ticks_since_reset;
    std::ofstream outputFile;
    Position global_position;
    double distanceMax;
    int trialNumber;
    double speedSetpoint;
    double phaseSetpoint;
    double rearPositionMax, tiltMax, tiltMin;
    long rearTicksAtMax, tiltTicksAtMax, tiltTicksAtMin;

private:
    double phaseController(double, double);
    double getPositionError(double, double);
    double addPositions(double, double);
    void collectData(double, double, double);
};

// Simple 2D vector utility class for phase calculations
class Vector {
public:
    double _x, _y;

    Vector(double);
    Vector(double, double);

    double angle(Vector);
    double smallestAngle(Vector);
    double length();
    double dot(Vector);
    Vector rotate(double);
};
#endif /* ODE_ROBOTS_ROBOTS_LOCOKIT_ROSINTERFACECONTROLLER_H_ */
