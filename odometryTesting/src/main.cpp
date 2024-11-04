/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Masuk-14                                                  */
/*    Created:      10/2/2024, 2:11:27 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//hi lam this is carmen pls explain
#include "vex.h"
#include "motor_config.h"
#include "pids.h"

#pragma once

using namespace vex;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

struct cartesian 
{
  double xCoord = 0.0; 
  double yCoord = 0.0; 
};

cartesian toPolar(cartesian Cart, double averageOrient)
{
  //Gets coordinate (r, theta) in polar coordinates
  cartesian Car; 

  //(R, Theta) 
  Car.xCoord = sqrtf((Cart.xCoord) * (Cart.xCoord) + (Cart.yCoord) * (Cart.yCoord));

    Car.yCoord = atan2f(Cart.yCoord, Cart.xCoord);
 
  //Changing the angle as described
  Car.yCoord -= averageOrient;

  return Car;
}

cartesian toCartesian(cartesian Cart)
{
  //Gets Coordinates (x, y) in Cartesian Coordinates from (r, theta) or R^2

  //X-Coordinate = r times cos(theta), Y-Coordinate = r times sin(theta)

  //Stores the original r
  double r = Cart.xCoord;  
  Cart.xCoord = r * cosf(Cart.yCoord); 
  Cart.yCoord = r * sinf(Cart.yCoord); 

  return Cart; 
}

double leftAtReset = 0, rightAtReset = 0;
double xCoordinate = 0.0, yCoordinate = 0.0; 

double offsetX = 0.0, offsetY = 0.0; 

double changeL, changeR, changeS; 

double currTheta = 0, prevTheta = 0; 
//Wheel radius in inches (arbitrary)

//3 to 5 gear ratio 

double theta = 0.0; 

//Creates Three Structs representative of different points in the odometry code. 
cartesian Current;
cartesian DeltaChange; 
cartesian Offset;  

double sumX = 0, sumY = 0; 

//Tracks how far the robot has traveled during the last iteration
double prevL = 0, prevR = 0, prevS = 0; 

double avgOrientation; 
//Function should be used as a task
int odometry()
{

  double deltaTheta = 0.0; 
  //Distance from tracking center to left, right, center Tracking wheel (inches): THESE ARE ARBITRARY!

  //OK Values: sL = 3.8, sR = 3.85, sS = 6.021: sL not needed for anything
  double const sR = 4.57, sS = 6.021; 
  double currentL = 0, currentR = 0, currentS = 0;

  //Distance = circumference * PI * (position / 360) - Go from revolutions to degrees

while(true)
{

//Finds current distance travel in inches
 currentL = (RotationL.position(degrees) / 360) * (3.141592 * diameter) * gearRatio;
 currentR = (RotationR.position(degrees) / 360) * (3.141592 * diameter) * gearRatio;
 currentS = (RotationS.position(degrees) / 360) * (3.141592 * diameter) * gearRatio;

//Calculates change in drive distance
 changeL = currentL - prevL; 
 changeR = currentR - prevR; 
 changeS = currentS - prevS;
  
  //Updates previous variables
  prevTheta = theta;
  prevL = currentL; 
  prevR = currentR; 
  prevS = currentS;

 //Returns current rotation change relative to arc in radians
  theta = Inertial1.rotation() * (M_PI / 180);

  //Adding coterminal angles
  while (theta > M_PI)
  {
    theta -= M_TWOPI; 
  }
  while (theta < -M_PI)
  {
    theta += M_TWOPI;
  }

  //Finds how much we have changed since last iteration
  deltaTheta = theta - prevTheta; 

  //If the angle did not change at all or very small
  if (fabs(deltaTheta) <= 0.001)
  {

    //Inverse x and y axis on cases PI/2 and 3PI/2 with slight error tolerance
    if ((fabs(theta) >= M_PI_2 - 0.001 && fabs(theta) <= M_PI_2 + 0.001) || (fabs(theta) >= (3 * M_PI_2) - 0.001) && (fabs(theta) <= (3 * M_PI_2) + 0.001))
    {
      offsetX = changeR; 
      offsetY = changeS; 
    }

    //Normal x and y position tracking
    else
    {
      offsetX = changeS;
      offsetY = changeR; 
    }

  }  

  //Point turn (wheel encoders moving in inverse directinns)
  else if (fabs(changeL + changeR) <= 0.075)
  {
    offsetX = 0; 
    offsetY = 0; 
  }                      

  //We have a change in the angle
  else
  { 
     //Multiplies by scalar quantity 2sin(currTheta / 2)
    offsetX = (2.0 * sinf(deltaTheta / 2.0)) * ((changeS / deltaTheta)  + sS); 
    offsetY = (2.0 * sinf(deltaTheta / 2.0)) * ((changeR / deltaTheta) + sR); 
  }

  //Updates Struct to vector change in position (Directly adding 
  //Offset over time does not work since it does not account for current angle of robot.)

  Offset.xCoord = offsetX; 
  Offset.yCoord = offsetY; 

  //Average arc angle
  avgOrientation = prevTheta + (deltaTheta / 2.0);

  //Converts to polar coordinates upon changing angle
  DeltaChange = toPolar(Offset, avgOrientation);

  //Converts deltaChange back to Cartesian
  DeltaChange = toCartesian(DeltaChange); 

  //Calculates next position by adding two positional matrices together
  Current.xCoord += DeltaChange.xCoord; 
  Current.yCoord += DeltaChange.yCoord;

  //Prints stuff to the brain
  Brain.Screen.printAt(20, 120, "xCoordinate: %.3f", Current.xCoord); 
  Brain.Screen.printAt(20, 220, "yCoordinate: %.3f", Current.yCoord);


  //Refresh as to not hog CPU - REMEMBER TO CHANGE!
  wait(10, msec);
}

  return 0; 
}

void pre_auton(void) {
  RotationL.setPosition(0, degrees);
  RotationR.setPosition(0, degrees);
  RotationS.setPosition(0, degrees);

  LeftBackMotor.setPosition(0, degrees);
  LeftMiddleMotor.setPosition(0, degrees);
  LeftFrontMotor.setPosition(0, degrees);
  RightBackMotor.setPosition(0, degrees);
  RightMiddleMotor.setPosition(0, degrees);
  RightFrontMotor.setPosition(0, degrees);

  wait(20, msec); 

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

double getDistance(double x2, double y2, double x1, double y1)
{
  double deltaX2 = (x2 - x1) * (x2 - x1);
  double deltaY2 = (y2 - y1) * (y2 - y1); 

  return sqrtf(deltaX2 - deltaY2);
}

//Function determines additive (no special cases)
double determineQuad(double curX, double curY, double desX, double desY)
{
  //Ex: (3,5) to (6, 10)
  if (curX < desX && curY < desY)
  {
    //Do not need to add anything
    return 0.0; 
  }

  //Ex: (3,5) to (6, 2)
  else if (curX < desX && curY > desY)
  {
    //You need to add 90 degrees to the angle
    return 90.0; 
  }
  
  //Ex: (3,5) to (-3, -5)
  else if (curX > desX && curY > desY)
  {
    return 180.0;
  }

  //Ex: (3,5) to (-5, 8)
  else if (curX > desX && curY < desY)
  {
    return 270.0; 
  }

  return 0.0;
}

//Gives the special cases
double specialCases(double curX, double curY, double desX, double desY)
{
  if (curX == desX && curY != desY)
  {
    if (desX > curX)
    {
      return 90.0; 
    }
    else
    {
      return -90.0; 
    }
  }

  else if (curY == desY && curX != desY)
  {
    if (desY > curY)
    {
      return 0.0;
    }
    else
    {
      return 180.0; 
    }
  }

  //Points are equal for some reason
  else
  {
    return 0; 
  }
}

void driveToPoint(double desiredX, double desiredY, double velocit, int waity)
{ 
  double requiredAngle; 
  //Gives the current coordinates + info
  double currentX = Current.xCoord; 
  double currentY = Current.yCoord; 

  //Distance formula calculation
  double dist = getDistance(desiredX, desiredY, currentX, currentY);

  //Turn to Desired Angle

  //Do not consider special cases here
  if (currentX != desiredX && currentY != desiredY)
  {
    //Determines the additive for the final angle calculation
    double additive = determineQuad(currentX, currentY, desiredX, desiredY);

    //Gives the angle that we need to consider
    double dX = desiredX - currentX; 
    double dY = desiredY - currentY; 
    requiredAngle = atan2f(dY, dX) + additive; 
  }
  //Either desiredX and currentX are equal or desiredY and currentY are equal.
  else
  {
    requiredAngle = specialCases(currentX, currentY, desiredX, desiredY);
  }

  //Drive to the point
  turnPID(requiredAngle, velocit, waity);
  drivePID(dist, velocit, waity);
}

void autonomous(void) {

  //Have odometry constantly running

    vex::task odom(odometry);
    wait(55, sec); 
    vex::task sleep(odom);
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  float LSpeed, LogL; 
  float RSpeed, LogR;
  int count = 0;

  // User control code here, inside the loop
  while (1) {
    LSpeed = Controller1.Axis3.position(percent); 
    RSpeed = Controller1.Axis2.position(percent); 

    LogL = ((LSpeed * LSpeed) / 100);
    LogR = ((RSpeed * RSpeed) / 100);

  if (fabs(LogL) >= 1 || fabs(LogR) >= 1)
  {
    if (LSpeed < 0)
    {
      LogL = -LogL; 
    }
    if (RSpeed < 0)
    {
      LogR = -LogR;
    }
    LeftBackMotor.spin(forward, LogL, percent);
    RightBackMotor.spin(forward, LogR, percent);
    LeftFrontMotor.spin(forward, LogL, percent);
    RightMiddleMotor.spin(forward, LogR, percent);
    LeftMiddleMotor.spin(forward, LogL, percent); 
    RightFrontMotor.spin(forward, LogR, percent); 
  }
  else
  {
    LeftBackMotor.stop(vex::brakeType::coast);
    LeftFrontMotor.stop(vex::brakeType::coast); 
    LeftMiddleMotor.stop(vex::brakeType::coast);
    RightBackMotor.stop(vex::brakeType::coast); 
    RightMiddleMotor.stop(vex::brakeType::coast);
    RightFrontMotor.stop(vex::brakeType::coast);
  }

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
