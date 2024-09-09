/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       18602                                                     */
/*    Created:      6/24/2024, 6:01:50 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

brain Brain; 
controller Controller1 = controller(primary); 

digital_out pneu = digital_out(Brain.ThreeWirePort.B);

motor LeftFrontMotor = motor(PORT1, ratio6_1, false); 
motor LeftBackMotor = motor(PORT3, ratio6_1, false);
motor RightFrontMotor = motor(PORT4, ratio6_1, true);
motor RightBackMotor = motor(PORT2, ratio6_1, true); 

inertial Inertial1 = inertial(PORT3); 

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

void pre_auton(void) {
  LeftFrontMotor.setPosition(0, degrees); 
  RightFrontMotor.setPosition(0, degrees); 
  LeftBackMotor.setPosition(0, degrees);
  LeftFrontMotor.setPosition(0, degrees);

  Inertial1.calibrate(); 
  while (Inertial1.isCalibrating())
  {
    wait(20, msec); 
  }

  Inertial1.setHeading(0, degrees); 

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

float inchesToDegrees(float inches)
{
  const float wheelRadius = 3.25; 
  const float gear_ratio = 5/3; 

  float store = wheelRadius * (3.141592/360) * (gear_ratio);
  float degrees = inches / store;
  return degrees; 
}

void drivePID (double desiredPosition, int velocit)
{
  double kP = 0.5; 
  double kI; 
  double kD = 0.2; 

  double power = 0; 

  double error = 10;
  double prevError = 0;  

   LeftFrontMotor.setPosition(0, degrees); 
   LeftBackMotor.setPosition(0, degrees);
   RightFrontMotor.setPosition(0, degrees);
   RightBackMotor.setPosition(0, degrees); 

   double convertDeg = inchesToDegrees(desiredPosition); 


  while (error > 5)
  {
    //Tracks how far the robot has currently gone
    double encoderValue = (LeftFrontMotor.position(degrees) +
    LeftBackMotor.position(degrees) + RightFrontMotor.position(degrees) + 
    RightBackMotor.position(degrees)) / 4; 

    //Calculates the error and derivative
    error = convertDeg - encoderValue; 
    double derivative = error - prevError; 

    //Does the actual process of setting the velocity
    power = (error * kP) + (derivative * kD);

    //Makes the robot go forward
    LeftFrontMotor.spin(forward, power, percent); 
    LeftBackMotor.spin(forward, power, percent);
    RightFrontMotor.spin(forward, power, percent); 
    RightBackMotor.spin(forward, power, percent);

    prevError = error; 
    
    wait(20, msec); 
  }

  LeftFrontMotor.stop(vex::brakeType::hold); 
  LeftBackMotor.stop(vex::brakeType::hold);
  RightFrontMotor.stop(vex::brakeType::hold);
  RightBackMotor.stop(vex::brakeType::hold);
}

void turnPID (int turnTo, int velocit)
{
  double kP = 0.5; 
  double kI; 
  double kD = 0.2; 

  double power = 0; 

  double error = 10;
  double derivative; 
  double prevError = 0;  

   LeftFrontMotor.setPosition(0, degrees); 
   LeftBackMotor.setPosition(0, degrees);
   RightFrontMotor.setPosition(0, degrees);
   RightBackMotor.setPosition(0, degrees); 


  while (fabs(error) >= 0.1)
  {
    error = turnTo - Inertial1.rotation(degrees); 
    derivative = error - prevError; 

    power = (error * kP) + (derivative * kD); 

    LeftFrontMotor.spin(forward, power, percent); 
    LeftBackMotor.spin(forward, power, percent); 
    RightFrontMotor.spin(forward, -power, percent);
    RightBackMotor.spin(forward, -power, percent); 

    prevError = error; 
    wait(20, msec); 
  }

  LeftFrontMotor.stop(vex::brakeType::hold); 
  LeftBackMotor.stop(vex::brakeType::hold);
  RightFrontMotor.stop(vex::brakeType::hold);
  RightBackMotor.stop(vex::brakeType::hold);
}
void autonomous(void) {

  //Implementation of drivePID is (inches, velocity); 

  drivePID(25, 20);
  turnPID(90, 3); 
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
  float LSpeed; 
  float RSpeed;
  // User control code here, inside the loop
  while (1) {
    LSpeed = Controller1.Axis3.position(percent); 
    RSpeed = Controller1.Axis2.position(percent); 

  if (fabs(LSpeed) >= 10 || fabs(RSpeed) >= 10)
  {
    LeftBackMotor.spin(forward, LSpeed, percent); 
    LeftFrontMotor.spin(forward, LSpeed, percent);
    RightBackMotor.spin(forward, RSpeed, percent);
    RightFrontMotor.spin(forward, RSpeed, percent); 
  }
  else
  {
    LeftBackMotor.stop(vex::brakeType::hold);
    LeftFrontMotor.stop(vex::brakeType::hold); 
    RightBackMotor.stop(vex::brakeType::hold); 
    RightFrontMotor.stop(vex::brakeType::hold);
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
