#include "main.h"
#include "lemlib/api.hpp"
#include "robot_config.h"
#include "functions.h"

#pragma once

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
//Switch Back
static bool pressed = false;
int selection = 0; 
void on_center_button() {
	selection++; 
	if (selection == 0) {
		pros::lcd::clear_line(2);
		pros::lcd::set_text(2, "Prog Skills selected");
	} 
	else if (selection == 1) {
		pros::lcd::clear_line(2);
		pros::lcd::set_text(2, "Blue Ring Side selected");
	}
	else if (selection == 2) {
		pros::lcd::clear_line(2);
		pros::lcd::set_text(2, "Red Ring Side selected");
	}
	else if (selection == 3) {
		pros::lcd::clear_line(2);
		pros::lcd::set_text(2, "Red Goal Side selected");
	}
	else if (selection == 4) {
		pros::lcd::clear_line(2);
		pros::lcd::set_text(2, "Blue Goal Side selected");
	}
	else if (selection == 5) {
		pros::lcd::clear_line(2);
		pros::lcd::set_text(2, "Red Goal Non Rush");
	}
	else if (selection == 6) {
		pros::lcd::clear_line(2);
		pros::lcd::set_text(2, "Blue Goal Non Rush");
	}
	else
	{
		selection = 0; 
		pros::lcd::clear_line(2);
		pros::lcd::set_text(2, "Prog Skills selected");
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	chassis.calibrate();
	optical.set_led_pwm(100);
	rotationArm.set_position(0); 
	rotationLeft.set_position(0); 
	rotationRight.set_position(0); 
	rotationCenter.set_position(0);
	
	pros::lcd::initialize();

	pros::lcd::register_btn1_cb(on_center_button);

	
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */

void disabled() {
	
	}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selectionor
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
	  // set position to x:0, y:0, heading:0 
//X AND Y are swapped on jerryio for skills

	//Prog Skills Code
	if (selection == 0)
	{	
		//Sets robot starting pos
		chassis.setPose(0, -175, 0);
		matics(true); 

		//First ring
		intake.move_relative(-1500, 100); 
		pros::delay(500); 
		intake.move_relative(600, 100); 
		pros::delay(200);

		chassis.moveToPoint(0, -159, 1000);
		chassis.turnToHeading(-88, 1200); 

		pros::delay(1200);

		//Goes to first mobile goal
		drivePID(-1250, 0.42, 1000);
		drivePID(-300, 0.32, 350); 

		pros::delay(350);
		//Clamps 
		matics(false); 
		pros::delay(100);
		chassis.turnToHeading(0, 1000);
		pros::Task intakoe(intakey); 

		chassis.moveToPoint(22, -135, 1500);
		intake.move_relative(200, 100); 
		pros::Task intakoer(intakey); 
		pros::delay(700); 
		chassis.turnToHeading(85, 1000);
		chassis.moveToPoint(50.5, -144, 1800); 

		//Goes For Wall Stake
		chassis.turnToHeading(41, 1000); 
		chassis.moveToPoint(57, -116.5, 2200);

		pros::Task intakee(intakey);
		//Goes for corner  
		chassis.moveToPoint(54, -125, 2000, {.forwards = false}); 
		chassis.turnToHeading(170, 1000); 
		chassis.moveToPoint(58, -165, 2000); 
		chassis.moveToPoint(47, -176, 1500);
		pros::delay(1700); 
		//chassis.turnToHeading(135, 1000);

		intake.move_relative(300, 100);
		pros::Task intakoeyrr(intakey);  

		chassis.turnToHeading(-45, 1000, {.maxSpeed = 79}); 
		/*pros::delay(1000);
		drivePID(900, 1, 1000); 
		pros::delay(1200); */

		chassis.moveToPoint(63.5, -180, 1500, {.forwards = false, .maxSpeed = 90});  

		pros::delay(200);
		matics(true);
		intake.move_relative(300, 100);
		pros::delay(100); 

		//Change back to maxSpeed 85 if needed
		chassis.moveToPoint(0, -175, 1800, {.maxSpeed = 85}); 
		
		colorSortRed = true; 
		pros::Task sort4(colorSort);
		
		intake.move(-127); 

		/*colorSortRed = true; 
		pros::Task sort2(colorSort); */

		chassis.turnToHeading(-90, 1200); 
		//pros::delay(200);
		//Goes for second mogo - Go back to -174 if necessary
		chassis.moveToPoint(-18, -172, 2500, {.forwards = false, .maxSpeed = 85});
		chassis.moveToPoint(-23, -177, 1000, {.forwards =  false, .maxSpeed = 55}); 
		pros::delay(1000); 

		matics(false); 
		pros::delay(600);

		colorSortRed = false; 
		
		intake.move_relative(20, 100);
		
		colorSortRed = false; 
		pros::delay(100);
		//pros::Task intakoee(intakey);
   
//middle ring
		chassis.turnToHeading(12, 1000); 
		chassis.moveToPoint(-22, -140, 2000); 
		
		chassis.turnToHeading(-90, 1000);												
		chassis.moveToPoint(-45.5, -139, 2000); 
//corner rings
		chassis.turnToHeading(-41, 1000); 
		chassis.moveToPoint(-52, -118, 1800); 
 
		chassis.turnToHeading(-172, 1000);       
		chassis.moveToPoint(-50, -166, 2000);
		chassis.moveToPoint(-52, -182, 1000, {.maxSpeed = 100});
   
		chassis.turnToHeading(52, 1800);
		//Change back to 175 if necessary
		chassis.moveToPoint(-62, -185, 2000, {.forwards = false}); 
		pros::delay(2000);

	

		intake.move_relative(200, 100);
		

		matics(true);
		drivePID(200, 500, 0.7);

		pros::delay(500); 
		intake.move(-127); 
		chassis.moveToPoint(-45, -118.5, 1800);

		colorSortRed = true; 
		pros::Task sort1(colorSort);
		 

		//Remove if necessary

		//Change back to -28 x-axis if necessary
		chassis.moveToPoint(-26, -99, 1700);

		
		//Go back to 200 if necessary
		chassis.turnToHeading(200, 1000);
		
		/*
		
		//Third Mogo 
		chassis.moveToPoint(0, -94.5, 3500, {.forwards = false, .maxSpeed = 105});
		chassis.moveToPoint(5.8, -90, 1000, {.forwards = false, .maxSpeed = 90});
	
		pros::delay(1000);       
		matics(false);
		pros::delay(100); 

		colorSortRed = false; 
		intake.move(-127);



		pros::delay(800); 
		chassis.turnToHeading(-100, 1000, {.minSpeed = 20});

		intake.move_relative(50, 20); 
		intake.move(-127);  
		pros::delay(1000); 

		
		colorSortBlue = true; 
		pros::Task sortBlue(colorSort);

		lemlib::Pose pose = chassis.getPose();
		chassis.moveToPoint(pose.x - 49.5, pose.y + 10, 1800, {.minSpeed = 40}); 

		//chassis.moveToPoint(pose.x - 41, pose.y + 10, 1000, {.forwards = false, .minSpeed = 20}); 
		chassis.turnToHeading(20, 900, {.minSpeed = 20}); 

		pros::delay(900); 

		drivePID(700, 1, 700); 

		pros::delay(700);

		chassis.turnToHeading(139, 1000, {.minSpeed = 20}); 

		pros::delay(1000);  
		drivePID(-780, 0.6, 1100);
		intake.move_relative(100, 50); 
		pros::delay(1000);  
		matics(true); 

		drivePID(-650, 0.8, 800); 

		
		//Pushes final mobile goal 
		drivePID(2000, 0.5, 1200); 

		pros::delay(1000); 
		chassis.turnToHeading(-85.5, 800, {.minSpeed = 20}); 

		pros::delay(800); 

		drivePID(-4200, 2.1, 2700); 
		drivePID(800, 0.6, 1000); 

		//CODE ENDS HERE!
		*/
		
	}
	//Blue Ring Side - REVERSE! (NOT REVERSED YET)
	else if (selection == 1)
	{
		optical.set_led_pwm(70); 
		matics(true);
		rotationArm.set_position(-3520);
		//Sets robot starting pos
		
		chassis.setPose(0, 0, 134);

		//Gets alliance wall stake 
		drivePID(210, 0.5, 750); 

		desiredArmPos = -225; 
		pros::Task moveArm(armPID);
		pros::delay(500); 
		drivePID(120, 0.7, 200);
		pros::delay(200); 

		desiredArmPos = 0;
		pros::Task moveArmy(armPID);

		drivePID(-850, 1, 500);
		pros::delay(500);
		chassis.turnToHeading(83, 800); 

		//Gets the stacked rings
			intake.move(-127); 
		specialMatics(true); 

		lemlib::Pose pose = chassis.getPose();

	
		
		chassis.moveToPoint(pose.x + 19, pose.y + 3, 1100, {.maxSpeed = 70, .minSpeed = 20}); 
		

		pros::delay(800);
		specialMatics(false); 
		
		//specialMatics(false);
		/*pros::delay(450);
		intake.move_relative(50, 100); 
		//intake.move_relative(-800, 100); 
		//pros::delay(500);  
		chassis.moveToPoint(pose.x + 15.5, pose.y + 2, 1500, {.forwards = false}); 
		enableIntake = false; */
		//pros::delay(350); 
		pros::delay(400); 


		//Change back if necessary
		chassis.turnToHeading(152, 800, {.minSpeed = 20});
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

	
		
		pros::delay(400); 		

		//pros::delay(200);

		//drivePID(580, 0.8, 1000); 		
		
		
		pros::delay(200); 

		//Moves to mobile goal

		
		
		//Back to -6.5 if necessary
		//chassis.moveToPoint(-7.5, 21.5, 1500, {.forwards = false, .maxSpeed = 85, .minSpeed = 20}); 

		drivePID(370, 1, 500);

		pros::delay(100);  
		colorSortRed = true;
		pros::Task sort1(colorSort);

		
		drivePID(-1200, 0.75, 900); 
		chassis.moveToPoint(-15.5, 35.5, 800, {.forwards = false, .maxSpeed = 80});
		
		//chassis.moveToPoint(-6.2, 18.5, 1100, {.forwards = false, .maxSpeed = 80}); 
		
		pros::delay(800);
		matics(false);
		pros::delay(700); 
		colorSortRed = false; 
		//pros::delay(700); 
		specialMatics(false); 
		
		
		intake.move_relative(100, 60); 
		chassis.turnToHeading(-40, 800, {.minSpeed = 20});
		pros::delay(300); 
		enableIntake = true; 
		
		intake.move(-127);  

		pros::delay(100); 

		lemlib::Pose pose2 = chassis.getPose();
		pros::Task intaker(intakey); 
		//Moves to center rings
		chassis.moveToPoint(-35, 56, 1100, {.minSpeed = 10});

		chassis.moveToPoint(-32, 43, 900, {.forwards = false}); 
		chassis.turnToHeading(255, 900, {.minSpeed = 20}); 
		//chassis.turnToHeading(-70, 800, {.minSpeed = 20});  

		pros::delay(100); 
		 

		chassis.moveToPoint(-45.5, 37, 1200, {.minSpeed = 20}); 
		chassis.turnToHeading(-15.5, 600, {.minSpeed = 20}); 

		pros::delay(600); 
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
		drivePID(950, 1.2, 800); 

		//chassis.moveToPoint(-52, 59, 600, {.forwards = false, .minSpeed = 20}); 

		drivePID(-600, 1.5, 600);
		pros::delay(600);   
		chassis.turnToHeading(65, 500, {.minSpeed = 40});


		desiredArmPos = -175; 
		pros::Task moveArmye(armPID);

		pros::delay(500); 

		//Turn on color sort for other color
		colorSortBlue = true;
		pros::Task sort2(colorSort);
		
		//Touch Ladder
		drivePID(1900, 1.8, 800);
	}

	//Red Ring Side
	else if (selection == 2)
	{
		optical.set_led_pwm(70); 
		matics(true);
		rotationArm.set_position(-3520);
		//Sets robot starting pos
		
		chassis.setPose(0, 0, 134);

		//Gets alliance wall stake 
		drivePID(210, 0.5, 750); 

		desiredArmPos = -225; 
		pros::Task moveArm(armPID);
		pros::delay(500); 
		drivePID(120, 0.7, 200);
		pros::delay(200); 

		desiredArmPos = 0;
		pros::Task moveArmy(armPID);

		drivePID(-850, 1, 500);
		pros::delay(500);
		chassis.turnToHeading(83, 800); 

		//Gets the stacked rings
			intake.move(-127); 
		specialMatics(true); 

		lemlib::Pose pose = chassis.getPose();

	
		
		chassis.moveToPoint(pose.x + 19, pose.y + 3, 1100, {.maxSpeed = 70, .minSpeed = 20}); 
		

		pros::delay(800);
		specialMatics(false); 
		
		//specialMatics(false);
		/*pros::delay(450);
		intake.move_relative(50, 100); 
		//intake.move_relative(-800, 100); 
		//pros::delay(500);  
		chassis.moveToPoint(pose.x + 15.5, pose.y + 2, 1500, {.forwards = false}); 
		enableIntake = false; */
		//pros::delay(350); 
		pros::delay(400); 


		//Change back if necessary
		chassis.turnToHeading(152, 800, {.minSpeed = 20});
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

	
		
		pros::delay(400); 		

		//pros::delay(200);

		//drivePID(580, 0.8, 1000); 		
		
		
		pros::delay(200); 

		//Moves to mobile goal

		
		
		//Back to -6.5 if necessary
		//chassis.moveToPoint(-7.5, 21.5, 1500, {.forwards = false, .maxSpeed = 85, .minSpeed = 20}); 

		drivePID(370, 1, 500);

		pros::delay(100);  
		colorSortRed = true;
		pros::Task sort1(colorSort);

		
		drivePID(-1200, 0.75, 900); 
		chassis.moveToPoint(-15.5, 35.5, 800, {.forwards = false, .maxSpeed = 80});
		
		//chassis.moveToPoint(-6.2, 18.5, 1100, {.forwards = false, .maxSpeed = 80}); 
		
		pros::delay(800);
		matics(false);
		pros::delay(700); 
		colorSortRed = false; 
		//pros::delay(700); 
		specialMatics(false); 
		
		
		intake.move_relative(100, 60); 
		chassis.turnToHeading(-40, 800, {.minSpeed = 20});
		pros::delay(300); 
		enableIntake = true; 
		
		intake.move(-127);  

		pros::delay(100); 

		lemlib::Pose pose2 = chassis.getPose();
		pros::Task intaker(intakey); 
		//Moves to center rings
		chassis.moveToPoint(-35, 56, 1100, {.minSpeed = 10});

		chassis.moveToPoint(-32, 43, 900, {.forwards = false}); 
		chassis.turnToHeading(255, 900, {.minSpeed = 20}); 
		//chassis.turnToHeading(-70, 800, {.minSpeed = 20});  

		pros::delay(100); 
		 

		chassis.moveToPoint(-45.5, 37, 1200, {.minSpeed = 20}); 
		chassis.turnToHeading(-15.5, 600, {.minSpeed = 20}); 

		pros::delay(600); 
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
		drivePID(950, 1.2, 800); 

		//chassis.moveToPoint(-52, 59, 600, {.forwards = false, .minSpeed = 20}); 

		drivePID(-600, 1.5, 600);
		pros::delay(600);   
		chassis.turnToHeading(65, 500, {.minSpeed = 40});


		desiredArmPos = -175; 
		pros::Task moveArmye(armPID);

		pros::delay(500); 

		//Turn on color sort for other color
		colorSortBlue = true;
		pros::Task sort2(colorSort);
		
		//Touch Ladder
		drivePID(1900, 1.8, 800);

	}
	//Red Goal Rush
	else if (selection == 3)
	{
		chassis.setPose(0, 0, 180); 
		chassis.moveToPoint(-7, 37 ,1800, {.forwards = false});
		pros::delay(200);
		chassis.moveToPoint(-12, 41.5,800, {.forwards = false, .maxSpeed = 85});
		pros::delay(500);
		matics(true);
		pros::delay(500);

		pros::Task intakeyr(intakey); 
		pros::delay(200);
		//chassis.moveToPoint(-13, 38, 1000, {.forwards = false}); 
		drivePID(200, 1, 1000); 
		pros::delay(1000);
		chassis.turnToHeading(270, 1600);
		pros::delay(1000);
		matics(false);
		chassis.moveToPoint (2,37,1000,{.forwards = false});

		intake.move_relative(1000, 200); 
		
		chassis.turnToHeading(115, 1500);
		
		chassis.moveToPoint (-29,32,2000, {.forwards = false});
		chassis.moveToPoint (-32,30,2000, {.forwards = false, .maxSpeed = 65});

		//chassis.moveToPose(-8,45.5, 90, 2000,{.forwards = true});
		pros::delay(500);
		matics(true);
		pros::delay(500);
		chassis.turnToHeading(90,1500);
		pros::Task intakeyrr(intakey); 
		chassis.moveToPoint(-4.5,28,2500, {.minSpeed = 70});
		pros::delay(400); 
		chassis.turnToHeading (240, 1500);

		intake.move_relative(500, 100); 
		pros::Task intakeyrre(intakey); 

		//Changed from (-65, 7) to (-65, 15)
		chassis.moveToPoint(-62, -7, 2800, {.forwards = true}); 
		specialMatics(true);
		chassis.moveToPoint(-69, -14, 2800, {.forwards = true, .maxSpeed = 80}); 
		pros::delay(2000); 
		specialMatics(false);
	}

	//Blue Goal Rush
	else if (selection == 4)
	{
		chassis.setPose(0, 0, 180); 
		chassis.moveToPoint(7, 37 ,1800, {.forwards = false});
		pros::delay(200); 
		chassis.moveToPoint(12, 41.5, 500, {.forwards = false, .maxSpeed = 85});
		pros::delay(500);
		matics(true);
		pros::delay(500);

		//Remove these delays
		pros::Task intakeyr(intakey); 
		pros::delay(1200);
		//chassis.moveToPoint(13, 38, 1000, {.forwards = false}); 
		drivePID(200, 1, 1000); 
		pros::delay(1000);
		chassis.turnToHeading(-270, 1600);
		pros::delay(1600);
		matics(false);

		chassis.moveToPoint (2,37,1000,{.forwards = false});
		
		intake.move_relative(1000, 200);
		pros::delay(200);  
		
		chassis.turnToHeading(-115, 1500);
		
		//Y-coordinates were 34 and 32
		chassis.moveToPoint (29,31,2000, {.forwards = false});
		chassis.moveToPoint (32,29,2000, {.forwards = false, .maxSpeed = 65});

		//chassis.moveToPose(-8,45.5, 90, 2000,{.forwards = true});
		pros::delay(500);
		matics(true);
		pros::delay(500);
		chassis.turnToHeading(-90,1500);
		pros::Task intakeyrr(intakey); 
		chassis.moveToPoint(3,28,2500, {.minSpeed = 70});
		pros::delay(400); 
		chassis.turnToHeading (-240, 1500);

		
		chassis.moveToPoint(62, -4, 2800, {.forwards = true}); 
		specialMatics(true);
		chassis.moveToPoint(69, -8, 2800, {.forwards = true, .maxSpeed = 70}); 
		pros::delay(2000);
		specialMatics(false);
		
	}

	//Red goal side (Non rush)
	else if (selection == 5)
	{
	//Sets robot starting pos
		rotationArm.set_position(-3520);		
		chassis.setPose(0, 0, -134);

		pros::delay(1000);

	//Gets preload on to alliance wall stake 
		drivePID(210, 0.6, 600); 
		desiredArmPos = -275; 
		pros::Task moveArm(armPID);
		pros::delay(900); 
	//Moves arm back down to loading position
		desiredArmPos = 0;
		pros::Task moveArmy(armPID);
		drivePID(-850, 1, 500);
		pros::delay(100);
	//Goes to Ring stack

	

		chassis.turnToHeading(-75, 1000); 
		specialMatics(true); 
		intake.move(-127); 
		lemlib::Pose pose = chassis.getPose();
		
		drivePID(850, 0.5, 3000);
		colorSortRed = true;
		pros::delay(200);
		specialMatics(false); 
		pros::delay(200);
		drivePID(100, 1, 1000);
	//Turns to pull top ring down 


	
		chassis.turnToHeading(-180, 900, {.minSpeed = 20});
	//Resets the arms to loading position for consistency

	
	
	intake.move(-127);
		drivePID(170, 0.9, 600);
		pros::delay(200);


			colorSortRed = true; 
	pros::Task sort1(colorSort);

		drivePID(295, 1, 600);
		pros::delay(500);
		//desiredArmPos = -275; 
		
		//pros::rtos::Task moveArm(armPID);
		pros::delay(100);
		drivePID(-250, 1, 400);
		//desiredArmPos = 0; 
		
		//pros::rtos::Task moveArmy(armPID);
		
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
		chassis.moveToPoint(5.5, 23, 1800, {.forwards = false, .maxSpeed = 85}); 
		chassis.moveToPoint(6.2, 29.5, 1000, {.forwards = false, .maxSpeed = 75}); 

		
		//chassis.moveToPoint(-6.2, 18.5, 1100, {.forwards = false, .maxSpeed = 80}); 

		
		pros::delay(1000);
		matics(true);
		pros::delay(700); 

		colorSortRed = false; 

		intake.move(-127); 
		
		//gets third ring
		chassis.turnToHeading(85, 800, {.minSpeed = 20});
		
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
		pros::delay(200); 
		//pros::delay(100); 
		drivePID(1500, 1.2, 1500);
		pros::delay(600);

		/*
		Ladder Touch
		chassis.turnToHeading(-69, 800, {.minSpeed = 20});

		drivePID(2000, 1.5, 1000); 
	
		/*desiredArmPos = -180; 
		pros::Task moveArmye(armPID);*/

		//Enable after elims
		chassis.turnToHeading(-225, 900, {.minSpeed = 20});

		pros::delay(200); 

		chassis.moveToPoint(55, -5, 900, {.minSpeed = 20}); 
		pros::delay(1000);
		drivePID(-1000, 1, 1000); 

		
	}
	//Blue goal side (non-rush)
	else if (selection == 6)
	{
		//Sets robot starting pos
		rotationArm.set_position(-3520);		
		chassis.setPose(0, 0, 134);
 
	//Gets preload on to alliance wall stake 
		drivePID(210, 0.6, 600); 
		desiredArmPos = -255; 
		pros::Task moveArm(armPID);
		pros::delay(900); 
	//Moves arm back down to loading position
		desiredArmPos = 0;
		pros::Task moveArmy(armPID);
		drivePID(-850, 1, 500);
		pros::delay(100);
	//Goes to Ring stack
		chassis.turnToHeading(75, 1000); 
		specialMatics(true); 
		intake.move(-127); 
		lemlib::Pose pose = chassis.getPose();
		
		drivePID(850, 0.5, 3000);
		
		pros::delay(200);
		specialMatics(false); 
		pros::delay(200);
		drivePID(100, 1, 1000);
	//Turns to pull top ring down 

	intake.move(-127);

	
		chassis.turnToHeading(180, 900, {.minSpeed = 20});
	//Resets the arms to loading position for consistency

		
		drivePID(170, 0.9, 600);
		pros::delay(200);

//DISABLE IF NECESSARY
		colorSortBlue = true; 
	pros::Task sort1(colorSort);

	intake.move(-127); 
		drivePID(295, 1, 600);
		pros::delay(500);
		//desiredArmPos = -275; 
		
		//pros::rtos::Task moveArm(armPID);
		pros::delay(100);
		drivePID(-250, 1, 400);
		//desiredArmPos = 0; 
		
		//pros::rtos::Task moveArmy(armPID);
		
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
		chassis.moveToPoint(-5.5, 23.5, 1800, {.forwards = false, .maxSpeed = 85}); 
		chassis.moveToPoint(-6.2, 29.5, 1000, {.forwards = false, .maxSpeed = 75}); 

		
		//chassis.moveToPoint(-6.2, 18.5, 1100, {.forwards = false, .maxSpeed = 80}); 

		
		pros::delay(1000);
		matics(true);
		pros::delay(900); 

		colorSortBlue = false; 

		intake.move(-127); 
		
		//gets third ring
		chassis.turnToHeading(-76, 800, {.minSpeed = 20});
		
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
		pros::delay(800); 
		//pros::delay(100); 
		drivePID(1500, 1.2, 1500);
		pros::delay(600);

		//turns to ladder - change back to 79 if necessary
		chassis.turnToHeading(79, 900, {.minSpeed = 20});
		drivePID(2500, 0.95, 1500);

		desiredArmPos = -150;
		pros::Task moveArmue(armPID); 
		

		/*pros::delay(200); 
			chassis.turnToHeading(225, 900, {.minSpeed = 20});

		chassis.moveToPoint(-55, -5, 1000, {.minSpeed = 2});

		pros::delay(1200); 

		drivePID(-2000, 1, 1000);  */
		//Enable after elims
		

		
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	bool mat = true; 
	enableIntake = false; 

	while (true) {
		 // Prints to brain
		 pros::lcd::print(3, "rotationPos: %i", rotationArm.get_position() / 100); 

		//Tank Drive
		int left = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int right = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y); 

		//Log Drive with Controller Deadzone
		double logLeft = ((left * left) / 127.0); 
		double logRight = ((right * right) / 127.0); 

		if (fabs(logLeft) >= 0.785 || fabs(logRight) >= 0.785)
		{
			if (left < 0)
			{
				logLeft = -logLeft; 
			}
			if (right < 0)
			{
				logRight = -logRight; 
			}
			left_motors.move(logLeft);
			right_motors.move(logRight);
		}
		else
		{
			left_motors.move(0); 
			right_motors.move(0); 
		}
		
		//Intake
		if (master.get_digital(DIGITAL_L2))
		{
			intake.move_velocity(200);
		}
		//Outake
		else if (master.get_digital(DIGITAL_L1))
		{
			intake.move_velocity(-200); 
		}
		else
		{
			intake.move_velocity(0);
			intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}

		//Goes to  spot for intaking
		if (master.get_digital(DIGITAL_R2))
		{
			//Go back to armPID if needed
			desiredArmPos = -35;
			pros::Task moveArm(armPID);
			pros::delay(150);
		}
		//Gets neutral wall stake
		else if (master.get_digital(DIGITAL_R1))
		{
			desiredArmPos = -155.5; 
			pros::Task moveArm(armPID);
			pros::delay(150); 
		}
		//Goes all the way down
		else if (master.get_digital(DIGITAL_DOWN))
		{
			desiredArmPos = 0; 
			pros::Task moveArm(armPID);
			pros::delay(150);  
		}
		else if (master.get_digital(DIGITAL_RIGHT))
		{
			armLeft.move_velocity(50); 
			armRight.move_velocity(50); 
		}
		else
		{
			armLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			armRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); 
			armLeft.move_velocity(0);
			armRight.move_velocity(0);
		}

		//Goes all the way up
		if (master.get_digital(DIGITAL_UP))
		{
			desiredArmPos = -225; 
			pros::Task moveArm(armPID);
			pros::delay(200); 	
		}
		
		if (master.get_digital(DIGITAL_B))
		{
			pneu.set_value(mat);
			master.print(0, 0, "Clamped: %s", mat ? "Yes": "Noo"); 
			pros::delay(300);
			mat = !mat;
		}

		if (master.get_digital(DIGITAL_X))
		{
			pneuAuto.set_value(mat);
			pros::delay(300);
			mat = !mat;
		}

		//Resets arm
		if (master.get_digital(DIGITAL_LEFT))
		{
			rotationArm.set_position(100); 
			pros::delay(200); 
		}
		pros::delay(20);                               // Run for 20 ms then update
	}
}