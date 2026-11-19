#ifndef NAVIGATION_LOGIC_H
#define NAVIGATION_LOGIC_H

#include "RobotInterface.h"

// Compute the Mecanum velocity vector (vx, vy, omega) by fusing the 3 front
// trapezoid Ultrasonic sensors with the 2D RPLIDAR C1 sector clearances.
MotorCommand computeAutonomousMove(const SensorData& sensor_in);

#endif // NAVIGATION_LOGIC_H
