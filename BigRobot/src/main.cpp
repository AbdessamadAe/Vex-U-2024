/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "objectDtection.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
motor leftMotorA = motor(PORT1, ratio18_1, false);
motor leftMotorB = motor(PORT11, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT12, ratio18_1, true);
motor rightMotorB = motor(PORT4, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
controller Controller1 = controller(primary);
distance frontDistance = distance(PORT19);
brain::lcd screen = vex::brain::lcd();
vision visionSensor = vision(PORT10);


// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

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
  double dist = 100000;
  // User control code here, inside the loop
  while (1)
  {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    /*---------------------------------------------------------------------------*/
    /*                              Drivetrain Control                            */
    /*---------------------------------------------------------------------------*/
    // Axis4 = Left/Right on the left joystick
    // Axis2 = Up/Down on the right joystick
    // controller1.Axis4.position() is for forward/backward movement
    // The controller1.Axis4.position() is preceeded by a minus sign to reverse
    // the direction of the right motors. This is so that the robot turns for e.g right when
    // the joystick is pushed to right by rotating the righ wheels backwords and the
    // left wheels forwards.
    

    //screen.clearScreen();
    dist = frontDistance.objectDistance(mm);
    screen.print("Dist: %f", dist);
    screen.newLine();

    if(dist > 170){
      RightDriveSmart.spin(vex::directionType::fwd,
                          Controller1.Axis2.position() - Controller1.Axis4.position(),
                          vex::velocityUnits::pct);
      LeftDriveSmart.spin(vex::directionType::fwd,
                         Controller1.Axis2.position() + Controller1.Axis4.position(),
                         vex::velocityUnits::pct);
    }

    else if(Controller1.Axis2.position() < 0) {
      RightDriveSmart.spin(vex::directionType::fwd,
                          Controller1.Axis2.position() - Controller1.Axis4.position(),
                          vex::velocityUnits::pct);
      LeftDriveSmart.spin(vex::directionType::fwd,
                         Controller1.Axis2.position() + Controller1.Axis4.position(),
                         vex::velocityUnits::pct);
    }
    else {
      RightDriveSmart.stop(vex::brakeType::brake);
      LeftDriveSmart.stop(vex::brakeType::brake);
    }
    
    visionSensor.takeSnapshot(GREENTRIBALL);
    if(visionSensor.largestObject.exists){
      screen.print("Green Triball X: %d ", visionSensor.largestObject.centerX);
      screen.print("Y: %d ", visionSensor.largestObject.centerY);
      screen.print("W: %d ", visionSensor.largestObject.width);
      screen.print("H %d ", visionSensor.largestObject.height);
      screen.newLine();
    }
    
    

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
    screen.print("========================================");
    screen.newLine();
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
