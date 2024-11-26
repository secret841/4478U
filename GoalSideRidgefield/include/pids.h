#include "vex.h"
#include "motor_config.h"
#pragma once

using namespace vex;

void drivePID (double driveInches, int velocit, int waity)
{
  double velCap = 1; 
  double kP = 1.65; 
  double kI; 
  double kD = 0.95; 

  int waitTime = 0; 

  double power = 0; 

  double error = 10;
  double prevError = 0;  

  double currentL = (RotationL.position(degrees) / 360) * (3.141592 * diameter) * gearRatio;
double currentR = (RotationR.position(degrees) / 360) * (3.141592 * diameter) * gearRatio;

    double encoderValue = (currentL + currentR) / 2; 

    //Correctly sets error
    driveInches += encoderValue; 


  while (fabs(error) >= 0.2 && waitTime <= waity)
  {

     currentL = (RotationL.position(degrees) / 360) * (3.141592 * diameter) * gearRatio;
    currentR = (RotationR.position(degrees) / 360) * (3.141592 * diameter) * gearRatio;
    //Tracks how far the robot has currently gone
    encoderValue = (currentL + currentR) / 2; 

    //Calculates the error and derivative
    
    error = driveInches - encoderValue;
    double derivative = error - prevError; 

    //Does the actual process of setting the velocity
    power = (error * kP) + (derivative * kD);

  //Gradual Acceleration
    if (power >= velCap)
    {
      power = velCap;
    }

    //Makes the robot go forward
    LeftFrontMotor.spin(forward, power * velocit * 0.01, voltageUnits::volt); 
    LeftMiddleMotor.spin(forward, power * velocit * 0.01, volt); 
    LeftBackMotor.spin(forward, power * velocit * 0.01, volt);
    RightFrontMotor.spin(forward, power * velocit * 0.01, volt); 
    RightMiddleMotor.spin(forward, power * velocit * 0.01, volt);
    RightBackMotor.spin(forward, power * velocit * 0.01, volt);

    prevError = error; 
    waitTime += 20;
    //Brain.Screen.printAt(250, 120, "error: %.3f", error); 

  velCap += 1.5; 
    vex::wait(20, msec); 
  }

  LeftFrontMotor.stop(vex::brakeType::hold);
  LeftMiddleMotor.stop(vex::brakeType::hold);  
  LeftBackMotor.stop(vex::brakeType::hold);
  RightFrontMotor.stop(vex::brakeType::hold);
  RightMiddleMotor.stop(vex::brakeType::hold);
  RightBackMotor.stop(vex::brakeType::hold);
}

void armPID(double desiredPosition, int velocit, int waity)
{
  double kP = 0.8, kI, kD = 1;
  double power; 
  int waitTime = 0;

  double error = 10;
  double prevError = 0;  
  double derivative = 0;

  int velCap = 1; 

  while (fabs(error) > 2 && waitTime < waity)
  {
    //Calculates preliminaries
    double error = desiredPosition - RotationArm.position(degrees); 
    derivative = error - prevError; 

    if (power >= velCap)
    {
      power = velCap; 
    }
    power = (error * kP) + (derivative * kD); 

    Grabber.spin(forward, power * velocit * 0.01, percent);

    prevError = error; 
    waitTime += 20; 
    velCap += 4; 

    vex::wait(20, msec);
  }  

  Grabber.stop(vex::brakeType::hold); 
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
   LeftMiddleMotor.setPosition(0, degrees); 
   LeftBackMotor.setPosition(0, degrees);
   RightFrontMotor.setPosition(0, degrees);
   RightMiddleMotor.setPosition(0, degrees);
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

void matics(bool go)
{
  pneu.set(go); 
  pneu2.set(go);
}

void moveFWD(int lAmount, int rAmount, int velocit)
{
  LeftFrontMotor.setVelocity(velocit, percent);
  LeftMiddleMotor.setVelocity(velocit, percent); 
  LeftBackMotor.setVelocity(velocit, percent);
  RightFrontMotor.setVelocity(velocit, percent);
  RightMiddleMotor.setVelocity(velocit, percent); 
  RightBackMotor.setVelocity(velocit, percent);

  LeftFrontMotor.spinFor(forward, lAmount, degrees, false);
  LeftMiddleMotor.spinFor(forward, lAmount, degrees, false); 
  LeftBackMotor.spinFor(forward, lAmount, degrees, false);
  RightFrontMotor.spinFor(forward, rAmount, degrees, false);
  RightMiddleMotor.spinFor(forward, rAmount, degrees, false);
  RightBackMotor.spinFor(forward,rAmount, degrees, false);
}
