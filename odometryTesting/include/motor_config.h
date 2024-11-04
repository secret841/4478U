//Motor configurations   

#include "vex.h"
#pragma once

using namespace vex;

double const diameter = 2.0; 
double const gearRatio = 0.6;

brain Brain; 
controller Controller1 = controller(primary); 

digital_out pneu = digital_out(Brain.ThreeWirePort.A);
digital_out pneu2 = digital_out(Brain.ThreeWirePort.B);
limit Limit1 = limit(Brain.ThreeWirePort.C);
digital_out pneuAuto = digital_out(Brain.ThreeWirePort.D);

motor LeftFrontMotor = motor(PORT10, ratio6_1, true); 
motor LeftMiddleMotor = motor(PORT9, ratio6_1, true);
motor LeftBackMotor = motor(PORT8, ratio6_1, true);
motor RightFrontMotor = motor(PORT11, ratio6_1, false);
motor RightBackMotor = motor(PORT12, ratio6_1, false); 
motor RightMiddleMotor = motor(PORT13, ratio6_1, false);
motor Intake = motor(PORT5, ratio6_1, false);
motor Grabber = motor(PORT7, ratio6_1, false);

inertial Inertial1 = inertial(PORT2); 


//REMEMBER TO CHANGE THESE PORT VALUES
rotation RotationL = rotation(PORT17); 
rotation RotationR = rotation(PORT15); 
rotation RotationS = rotation(PORT16); 

// A global instance of competition
competition Competition;