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
#include <cmath>

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
  if(Armmotor.position() == 0){
    Armmotor.spinFor(directionType::fwd, 3 * 360, rotationUnits::deg);
    Flywheel.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
  }
  

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
  if(Armmotor.position() >= 360*2.98){
    Armmotor.spinFor(directionType::fwd, -2.5 * 360, rotationUnits::deg);
  }
  
  Armmotor.setBrake(brakeType::hold);
}

/*---------------------------------------------------------------------------*/
/*                            Autonomous Functions                           */

int current_motor_angle_left = 0;
int current_motor_angle_right = 0;
float d = 0;
float rw = 140;
float wheeldiam = 101.6;
float dtheta = 0;
int stop = 0;

struct
{
  int X = 0;
  int Y = 0;
  float theta = 0.0;
} position;

void track_location()
{
  // the distance between the front center and the right and left wheel
  dtheta = 0;

  // get the distance travelled by the left and right wheel by getting the change in angle then to mm
  float dr = (rightMotorA.position(deg) - current_motor_angle_right) * (M_PI * wheeldiam) / 360;
  float dl = (leftMotorA.position(deg) - current_motor_angle_left) * (M_PI * wheeldiam) / 360;

  // calculating the change in the orientation of the robot
  dtheta = (dr - dl) / (2 * rw);
  // calulating the distance travelled by the entire robot

  if (std::abs(dtheta) <= 0.008)
  {
    d = (dr + dl) / 2;
  }
  else
  {
    d = 2 * (dr / dtheta + rw) * sin(dtheta / 2);
    position.theta += dtheta;
  }

  position.X += d * cos(position.theta);
  position.Y += d * sin(position.theta);

  Brain.Screen.printAt(10, 120, "X: %d", position.X);
  Brain.Screen.printAt(10, 150, "Y: %d", position.Y);
  Brain.Screen.printAt(10, 180, "Theta: %f", position.theta * 180 / M_PI);
  Brain.Screen.printAt(10, 210, "dtheta: %f", dtheta * 180 / M_PI);
}

void turnRobotToFace(float angle)
{
  Drivetrain.turnFor(angle * 180 / M_PI, degrees);
}

void moveRobotForward(float distance)
{
  Drivetrain.driveFor(distance, mm);
}

void moveToCoordinate(float targetX, float targetY)
{
  // Calculate the difference in position
  position.X = 0;
  position.Y = 0;
  position.theta = 0;

  float deltaX = targetX - position.X;
  float deltaY = targetY - position.Y;

  // Calculate the angle to turn
  float targetAngle = atan2(deltaY, deltaX);
  float angleToTurn = targetAngle - position.theta;

  // Normalize the angle
  while (angleToTurn > M_PI)
    angleToTurn -= 2 * M_PI;
  while (angleToTurn < -M_PI)
    angleToTurn += 2 * M_PI;

  if (stop == 1)
  {
    return;
  }

  // Turn the robot to face the target
  turnRobotToFace(angleToTurn);

  // Calculate the distance to move
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  // Move the robot to the target
  moveRobotForward(distance);

  // Update the position
  position.X = targetX;
  position.Y = targetY;
  position.theta = targetAngle;
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
  rightMotorA.setPosition(0, deg);
  leftMotorA.setPosition(0, deg);

  //track_location();

  /* current_motor_angle_left = leftMotorA.position(deg);
  current_motor_angle_right = rightMotorA.position(deg);
    */
  moveToCoordinate(1037, 393);
  moveToCoordinate(10, 393);
  stop = 1;
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
  LeftDriveSmart.setStopping(brakeType::hold);
  RightDriveSmart.setStopping(brakeType::hold);
  Drivetrain.setDriveVelocity(50, pct);
  Drivetrain.setTurnVelocity(25, pct);

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

    Controller2.ButtonX.released([]()
                                { flywheel(100); });
    Controller2.ButtonY.released([]()
                                { Stopflywheel(); });

    /*---------------------------------------------------------------------------*/
    /*                             Test Autonomous                             */
    Controller2.ButtonA.pressed([]()
                                { autonomous(); });

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
