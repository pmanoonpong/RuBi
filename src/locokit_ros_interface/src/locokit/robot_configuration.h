#ifndef LOCOKIT_SENSORS_MOTORS_DEFINITION_H
#define LOCOKIT_SENSORS_MOTORS_DEFINITION_H

// Define your sensors here
enum locokitSensor {
    FRONT_LEFT_SPEED = 0,
    FRONT_RIGHT_SPEED = 1,
    REAR_LEFT_SPEED = 2,
    REAR_RIGHT_SPEED = 3,
    NUMBER_SENSORS = 4
};

// And the motors
enum locokitMotor {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    REAR_LEFT = 2,
    REAR_RIGHT = 3,
    NUMBER_MOTORS = 4,
    FRONT_LEFT_ID = 5,
    FRONT_RIGHT_ID = 4,
    REAR_LEFT_ID = 8,
    REAR_RIGHT_ID = 19
};

// And more than likely you want to define this too
#define IP "192.168.2.2"
#define PORT 1241

#endif
