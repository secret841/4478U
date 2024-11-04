#include "vex.h"
#include "motor_config.h"
#pragma once

using namespace vex;

void drivePID (double driveInches, int velocit, int waity)
{
  int velCap = 5; 
  Brain.Screen.clearLine();
  double kP = 0.225; 
  double kI; 
  double kD = 0.455; 

  int waitTime = 0; 

  double power = 0; 

  double error = 10;
  double prevError = 0;  

  double desiredPosition = ((driveInches) / (diameter * M_PI * gearRatio)) * 360; 


  while (fabs(error) > 5 && waitTime <= waity)
  {
    //Tracks how far the robot has currently gone
    double encoderValue = (RotationL.position(degrees) + RotationR.position(degrees)) / 2; 

    //Calculates the error and derivative
    
    error = desiredPosition - encoderValue;
    double derivative = error - prevError; 

    //Does the actual process of setting the velocity
    power = (error * kP) + (derivative * kD);

  //Gradual Acceleration
    if (power >= velCap)
    {
      power = velCap;
    }

    //Makes the robot go forward
    LeftFrontMotor.spin(forward, power * velocit * 0.01, percent); 
    LeftMiddleMotor.spin(forward, power * velocit * 0.01, percent); 
    LeftBackMotor.spin(forward, power * velocit * 0.01, percent);
    RightFrontMotor.spin(forward, power * velocit * 0.01, percent); 
    RightMiddleMotor.spin(forward, power * velocit * 0.01, percent);
    RightBackMotor.spin(forward, power * velocit * 0.01, percent);

    prevError = error; 
    waitTime += 20; 

  velCap += 4; 
    vex::wait(20, msec); 
  }

  LeftFrontMotor.stop(vex::brakeType::hold);
  LeftMiddleMotor.stop(vex::brakeType::hold);  
  LeftBackMotor.stop(vex::brakeType::hold);
  RightFrontMotor.stop(vex::brakeType::hold);
  RightMiddleMotor.stop(vex::brakeType::hold);
  RightBackMotor.stop(vex::brakeType::hold);
}

void turnPID (int turnTo, int velocit, int waity)
{
  double kP = 0.625; 
  double kI; 
  double kD = 0.295; 

  double power = 0; 
  double f = 1; 
  int waitTime = 0; 

  double error = 10;
  double derivative; 
  double prevError = 0;  

   LeftFrontMotor.setPosition(0, degrees); 
   LeftBackMotor.setPosition(0, degrees);
   RightFrontMotor.setPosition(0, degrees);
   RightBackMotor.setPosition(0, degrees); 


  while (fabs(error) >= 0.1 && waitTime <= waity)
  {
    error = turnTo - Inertial1.heading(degrees);

  if (error <= -180)
  {
    error += 360;
  }
   derivative = error - prevError; 

    power = (error * kP) + (derivative * kD); 

    if (power >= velocit)
    {
      power *= (velocit * 0.1);
    }

    LeftFrontMotor.spin(forward, power, percent); 
    LeftMiddleMotor.spin(forward, power, percent);
    LeftBackMotor.spin(forward, power, percent); 
    RightFrontMotor.spin(forward, -power, percent);
    RightMiddleMotor.spin(forward, -power, percent);
    RightBackMotor.spin(forward, -power, percent); 

    prevError = error; 
    waitTime += 20;

    vex::wait(20, msec); 
  }
  LeftFrontMotor.stop(vex::brakeType::hold); 
  LeftMiddleMotor.stop(vex::brakeType::hold);
  LeftBackMotor.stop(vex::brakeType::hold);
  RightFrontMotor.stop(vex::brakeType::hold);
  RightMiddleMotor.stop(vex::brakeType::hold);
  RightBackMotor.stop(vex::brakeType::hold);
}

