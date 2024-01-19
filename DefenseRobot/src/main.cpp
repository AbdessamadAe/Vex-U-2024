/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
motor leftMotorA = motor(PORT10, ratio18_1, false);
motor leftMotorB = motor(PORT20, ratio18_1, false);
motor rightMotorA = motor(PORT1, ratio18_1, true);
motor rightMotorB = motor(PORT11, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
controller Controller2 = controller(primary);
motor FlywheelA = motor(PORT7, ratio18_1, false);
motor FlywheelB = motor(PORT8, ratio18_1, true);
motor_group Flywheel = motor_group(FlywheelA, FlywheelB);
motor Armmotor = motor(PORT9, ratio18_1, false);

int ra = 0;


/*---------------------------------------------------------------------------*/
/*                                Flywheel Function                           */
void flywheel(int speed)
{
  Armmotor.spinFor(directionType::fwd, 3*360, rotationUnits::deg);
  Flywheel.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
  
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Motor 1 Efficiency: %f", FlywheelA.efficiency(pct));
  Brain.Screen.newLine();
  Brain.Screen.print("Motor 2 Efficiency: %f", FlywheelB.efficiency(pct));
  Brain.Screen.newLine();
  Brain.Screen.print("Motor 1 Temp: %f", FlywheelA.temperature(celsius));
  Brain.Screen.newLine();
  Brain.Screen.print("Motor 2 Temp: %f", FlywheelB.temperature(celsius));
  Brain.Screen.newLine();  

}

void Stopflywheel()
{
  Flywheel.stop();
  Armmotor.spinFor(directionType::fwd, -2.5*360, rotationUnits::deg);
  Armmotor.setBrake(brakeType::hold);
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
bool RemoteControlCodeEnabled = true;

void pre_auton(void)
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

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

void autonomous(void)
{
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  // Field dimensions: 3657.6mm x 3657.6mm
  Drivetrain.turnFor(turnType::right, 90, rotationUnits::deg);
  Drivetrain.driveFor(directionType::fwd, 690 * M_PI / 2, distanceUnits::mm);
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

void usercontrol(void)
{
  Drivetrain.setStopping(brakeType::hold);

  // User control code here, inside the loop
  while (1)
  {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    RightDriveSmart.spin(vex::directionType::fwd, -Controller2.Axis3.position() + Controller2.Axis4.position(), pct);
    LeftDriveSmart.spin(vex::directionType::fwd, -Controller2.Axis3.position() - Controller2.Axis4.position(), pct);
    // ........................................................................

    // Testing autonomous functions

    // Field dimensions: 3657.6mm x 3657.6mm

    /*---------------------------------------------------------------------------*/
    /*                             Flyweel Control                               */
  
    Controller2.ButtonX.pressed([](){ flywheel(100); });
    Controller2.ButtonY.pressed([](){ Stopflywheel(); });

    
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);
  }
}
