//LPZ ROBOTS
#include <selforg/agent.h>
#include <selforg/abstractrobot.h>
#include <selforg/one2onewiring.h>

//STD
#include <iostream>
#include <signal.h>

//ROS
#include <ros/ros.h>

//PACKAGE
#include "controller/RosInterfaceController.h"
#include "locokit/locokit_TCP.h"


/**
 * @brief killed Flag to kill the program
 */
volatile sig_atomic_t killed(0);
void killer(int signal)
{
    killed = 1;
}

/**
 * @brief main
 */
int main(int argc, char** argv)
{
    // Register the Linux signal
    signal(SIGINT, killer);
    ros::init(argc, argv, "locokit_ros_interface");

    // Creates controller, wiring and robot
    AbstractController* controller = new LocoKitRosController("RosController");
    AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise(), true);
    Locokit_TCP* robot = new Locokit_TCP();

    // Creates the agent
    std::list<PlotOption> plotoptions;
    Agent* agent = new Agent(plotoptions);
    agent->init(controller, robot, wiring);

    // Init params
    double step(0);
    double noise(0.0);

    // Loop
    while (!killed && robot->isConnected() && ros::ok()) { //!stop
        ros::spin();
        agent->step(noise, step);
        std::cout << "Step: " << step << std::endl;
        step++;
    };

    // Ends the program
    delete agent;
    std::cout << "Terminating!" << std::endl;

    return 0;
}
