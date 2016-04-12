/*
 * LocoKitController.cpp
 *
 *  Created on: Dec 27, 2014
 *      Author: leon
 */

#include <assert.h>

#include "RosInterfaceController.h"

using namespace std;
using namespace matrix;

LocoKitRosController::LocoKitRosController(const std::string& name)
    : AbstractController(name, "1.0"){}

LocoKitRosController::~LocoKitRosController()
{

}

void LocoKitRosController::stepNoLearning(const sensor* sensors, int number_sensors, motor* motors, int number_motors)
{
}

void LocoKitRosController::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber)
{
}



void LocoKitRosController::init(int sensornumber, int motornumber, RandGen* randGen)
{
    nSensors = sensornumber;
    nMotors = motornumber;
    //outputFile.open("output.csv", std::ios::app);
    //outputFile << "phase, " << phaseSetpoint << ", vel," << speedSetpoint << std::endl;
}

int LocoKitRosController::getSensorNumber() const
{
    return nSensors;
}

int LocoKitRosController::getMotorNumber() const
{
    return nMotors;
}

bool LocoKitRosController::store(FILE* f) const
{
    Configurable::print(f, 0);
    return true;
}

bool LocoKitRosController::restore(FILE* f)
{
    Configurable::parse(f);
    return true;
}


