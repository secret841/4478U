/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Masuk-14                                                  */
/*    Created:      10/2/2024, 2:11:27 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "motor_config.h"
#include "pids.h"

#pragma once

int select = 0; 

using namespace vex;
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

//ODOM CODE - DO NOT TOUCH 
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

  //Stores the original r
  double r = Cart.xCoord;  
  Cart.xCoord = r * cosf(Cart.yCoord); 
  Cart.yCoord = r * sinf(Cart.yCoord); 

  return Cart; 
}

//Variables
double offsetX = 0.0, offsetY = 0.0; 

double currTheta = 0, prevTheta = 0; 

double theta = 0.0; 

//Tracks how far the robot has traveled during the last iteration
double changeL, changeR, changeS; 
double prevL = 0, prevR = 0, prevS = 0;

//Angle which robot moves at
double avgOrientation; 

//Creates Three Structs representative of different points in the odometry code. 
cartesian Current;
cartesian DeltaChange; 
cartesian Offset;  

//Function should be used as a task
int odometry()
{
  double deltaTheta = 0.0; 

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

  //Point turn (wheel encoders moving in inverse directinns)  
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
  else if (fabs(changeR + changeL) <= 0.1)
  {
    offsetX = 0; 
    offsetY = 0; 
  }
                  
  //We have a change in the angle (gives chord length)
  else
  { 
    offsetX = (2.0 * sinf(deltaTheta / 2.0)) * ((changeS / deltaTheta)  + sS); 
    offsetY = (2.0 * sinf(deltaTheta / 2.0)) * ((changeR / deltaTheta) + sR); 
  }

  //Updates Struct to vector change in position (Directly adding 
  //Offset over time does not work since it does not account for current angle of robot.)

  Offset.xCoord = offsetX; 
  Offset.yCoord = offsetY; 

  //Angle which robot moves at during arc
  avgOrientation = prevTheta + (deltaTheta / 2.0);

  //Converts to polar coordinates upon changing angle
  DeltaChange = toPolar(Offset, avgOrientation);

  //Converts deltaChange back to Cartesian
  DeltaChange = toCartesian(DeltaChange); 

  //Calculates next position by adding two positional matrices together
  Current.xCoord += DeltaChange.xCoord; 
  Current.yCoord += DeltaChange.yCoord;

  //Prints stuff to the brain
  //Brain.Screen.printAt(20, 120, "xCoordinate: %.3f", Current.xCoord); 
  //Brain.Screen.printAt(20, 220, "yCoordinate: %.3f", Current.yCoord);
  //Refresh loop
  wait(10, msec);
}

  return 0; 
}

//END OF ODOM CODE - PRE AUTON
void pre_auton(void) {
  RotationL.setPosition(0, degrees);
  RotationR.setPosition(0, degrees);
  RotationS.setPosition(0, degrees);
  RotationArm.setPosition(0, degrees);

  LeftBackMotor.setPosition(0, degrees);
  LeftMiddleMotor.setPosition(0, degrees);
  LeftFrontMotor.setPosition(0, degrees);
  RightBackMotor.setPosition(0, degrees);
  RightMiddleMotor.setPosition(0, degrees);
  RightFrontMotor.setPosition(0, degrees);

  while(Inertial1.isCalibrating())
  {
    wait(20, msec); 
  }
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

//MORE CODE RELATED TO ODOM - DO NOT TOUCH
double getDistance(double x2, double y2, double x1, double y1)
{
  double deltaX2 = (x2 - x1) * (x2 - x1);
  double deltaY2 = (y2 - y1) * (y2 - y1); 

  return sqrtf(deltaX2 + deltaY2);
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
  else if (curY == desY && curX != desX)
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

void driveToPoint(double desiredX, double desiredY, double velocit, int waity, bool reverse)
{ 
  double requiredAngle = Inertial1.rotation(); 
  //Gives the current coordinates + info
  double currentX = Current.xCoord; 
  double currentY = Current.yCoord; 

  //Distance formula calculation
  double dist = getDistance(desiredX, desiredY, currentX, currentY);

  //Turn to Desired Angle
double additive; 
  //Do not consider special cases here
  if (currentX != desiredX && currentY != desiredY)
  {
    //Determines the additive for the final angle calculation
    additive = determineQuad(currentX, currentY, desiredX, desiredY);

    //Gives the angle that we need to consider
    double dX = desiredX - currentX; 
    double dY = desiredY - currentY; 

    //Must Convert to Degrees for Inertial
    requiredAngle = (atan2f(dY, dX) * (180 / M_PI)) + additive; 
  }
  //Either desiredX and currentX are equal or desiredY and currentY are equal.
  else
  {
    double compareReqAngle = specialCases(currentX, currentY, desiredX, desiredY);

    if (compareReqAngle == 0 || compareReqAngle == 180)
    {
      requiredAngle = Inertial1.rotation();
    }
    else
    {
      requiredAngle = specialCases(currentX, currentY, desiredX, desiredY);
    }
  }

  //Drive to the point
  if (reverse)
  {
    dist = -dist;
  }

  //Optimizing angle finding algorithm

    if (fabs(Inertial1.rotation() - (requiredAngle - 360)) < fabs(requiredAngle))
    {
      requiredAngle -= 360;
    }
    turnPID(requiredAngle, velocit, waity);
    drivePID(dist, velocit, waity);

   //Brain.Screen.printAt(250, 120, "dist: %.3f", dist); 
  //Brain.Screen.printAt(250, 220, "requiredAngle: %.3f", requiredAngle);

  printf("requiredAngle: %.3f\n", requiredAngle); 

   
}


int amount = 500;
int intaker()
{
  Intake.setVelocity(100, percent); 
  Intake.spinFor(forward, amount, degrees);
  Intake.stop(vex::brakeType::hold); 
  return 0;
}

int armPID()
{
  double kP = 0.8, kI, kD = 1;
  double power; 
  int waitTime = 0;

  int velocit = 100; 
  int waity = 1500; 
  double desiredPosition = 23; 

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
  return 0; 
}


//END OF ODOM CODE
void autonomous(void) {
  Intake.setVelocity(100, percent); 

  //Have odometry constantly running (false parameter is reverse modifier)

  //Blue Auton
  if (select == 0)
  {
      vex::task odom(odometry);

      Brain.Screen.print("Blue Auton Selected"); 
    matics(true);

    drivePID(-10, 80, 1200);
    vex::wait(0.25, sec);
    //NO PID's to negate any dumb correction, also not required for this action to work.
    drivePID(-6, 60, 1500);
    matics(false);




  //Intake STACKED Rings (Red on Blue)
    amount = -5000;
      vex::task intakee(intaker);
     
      turnPID(93, 90, 1000);
      wait(14, sec);
      wait(0.1, sec);
      drivePID(14.5, 90, 1700);


    vex::task intakoe(intaker);
    vex::wait(1.4, sec);
      matics(true);
      drivePID(-6, 100, 1000);




  //GRAB Final mobile Goal
    turnPID(-14, 85, 1300);
    drivePID(-8, 75, 1600);
    wait(0.1, sec);
    drivePID(-3, 90, 1000);
    matics(false);

    drivePID(10, 100, 1000); 
    turnPID(90, 100, 1000);

    armPID(130, 100, 800); 
    drivePID(9, 100, 1000);


   
    vex::task sleep(odom);
  }

  //Red Auton
  else
  {
    vex::task odom(odometry);

      Brain.Screen.print("Blue Auton Selected"); 
    matics(true);

    drivePID(-10, 80, 1200);
    vex::wait(0.25, sec);
    //NO PID's to negate any dumb correction, also not required for this action to work.
    drivePID(-6, 60, 1500);
    matics(false);




  //Intake STACKED Rings (Red on Blue)
    amount = -5000;
      vex::task intakee(intaker);

      turnPID(93, 90, 1000);
      wait(14, sec);
      wait(0.1, sec);
      drivePID(14.5, 90, 1700);


    vex::task intakoe(intaker);
    vex::wait(1.4, sec);
      matics(true);
      drivePID(-6, 100, 1000);




  //GRAB Final mobile Goal
    turnPID(-14, 85, 1300);
    drivePID(-8, 75, 1600);
    wait(0.1, sec);
    drivePID(-3, 90, 1000);
    matics(false);

    drivePID(10, 100, 1000); 
    turnPID(90, 100, 1000);

    armPID(130, 100, 800); 
    drivePID(9, 100, 1000);


   
    vex::task sleep(odom);

  }
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
  bool toggle = false;
  bool logicL1 = false;

  bool pneumy = true;

  int count = 0;

  // User control code here, inside the loop
  while (1) {
    //Log Drive and Deadzone
    LSpeed = Controller1.Axis3.position(percent); 
    RSpeed = Controller1.Axis2.position(percent); 

    LogL = ((LSpeed * LSpeed) / 127);
    LogR = ((RSpeed * RSpeed) / 127);

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

  bool pressed = Controller1.ButtonL1.pressing();
  bool pressed2 = Controller1.ButtonL2.pressing(); 

  //If button L1 was pressed and logic is false
  if (pressed && !logicL1)
  {
    //Ensures if statement will not run again, sets toggle
    logicL1 = true; 
    toggle = !toggle; 
  }
  else if (!pressed)
  {
    //Ensures that if-statement is able to run again.
    logicL1 = false;
  }

  //Intake Code
  if (Controller1.ButtonL1.pressing())
  {
    Intake.spin(reverse, 100, percent); 
  }
  else if (Controller1.ButtonL2.pressing())
  {
    Intake.spin(forward, 100 , percent);
  }
  else
  {
    Intake.stop(vex::brakeType::hold);
  }

  //Lift Mechanism Control
  if (Controller1.ButtonR1.pressing() && RotationArm.position(degrees) < 133)
  {
    Grabber.spin(forward, 100, percent); 
  }
  else if (Controller1.ButtonR2.pressing() && RotationArm.position(degrees) >= 0)
  {
    Grabber.spin(reverse, 100, percent); 
  }
  else
  {
    Grabber.stop(vex::brakeType::hold);
  }



  //Clamp Control
  if (Controller1.ButtonB.pressing())
  {
    pneu.set(pneumy); 
    pneu2.set(pneumy);
     pneumy = !pneumy;
     //Break to prevent spamming
    vex::wait(300, msec);
  }

//Activates Auto Arm Pneu
  if (Controller1.ButtonY.pressing())
  {
    pneuAuto.set(pneumy); 
     pneumy = !pneumy;
     //Break to prevent spamming
    vex::wait(300, msec);
  }

  if (Controller1.ButtonA.pressing())
  {
    vex::task armo(armPID); 
    wait(300, msec);
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

// Main will set up the competition functions and callbacks.
int main() {
  Brain.Screen.drawRectangle(0, 0, 240, 240, blue);
  Brain.Screen.printAt(100, 100, "Blue Auton"); 
  Brain.Screen.drawRectangle(241, 0, 240, 240, red);
  Brain.Screen.printAt(325, 100, "Red Auton");
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
     if (Brain.Screen.pressing() && Brain.Screen.xPosition() >= 0 && Brain.Screen.xPosition() <= 240 && Brain.Screen.yPosition() >= 0 && Brain.Screen.yPosition() <= 240)
    {
        Brain.Screen.clearScreen(); 
        Brain.Screen.printAt(120, 120, "Blue Auton Selected");
        select = 0; 
        vex::wait(5, msec); 
    }
    else if (Brain.Screen.pressing() && Brain.Screen.xPosition() >= 241 && Brain.Screen.xPosition() <= 1000 && Brain.Screen.yPosition() >= 0 && Brain.Screen.yPosition() <= 240)
    {
        Brain.Screen.clearScreen(); 
        Brain.Screen.printAt(120, 120, "Red Auton Selected");
        select = 1; 
        vex::wait(5, msec); 
    }
    wait(100, msec);
  }
}
