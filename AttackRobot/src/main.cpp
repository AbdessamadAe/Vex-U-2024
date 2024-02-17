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
int GEAR_RATIO = 3; // 84/48;
float WHEEL_DIAMETER = 101.6;
float WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER / 2) * M_PI;
float TURN_ANGLE_MOTOR_RATIO = 5.4;

// define your global instances of motors and other devices here
motor leftMotorA = motor(PORT9, ratio18_1, false);
motor leftMotorB = motor(PORT1, ratio18_1, false);
motor rightMotorA = motor(PORT12, ratio18_1, true);
motor rightMotorB = motor(PORT2, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, WHEEL_CIRCUMFERENCE, 390, 315, mm, GEAR_RATIO);
controller Controller2 = controller(primary);
motor intakeMotorA = motor(PORT10, ratio18_1, false);
motor intakeMotorB = motor(PORT20, ratio18_1, true);
motor_group intake = motor_group(intakeMotorA, intakeMotorB);
motor shieldMotor = motor(PORT11, ratio18_1, false);

int shiledUp = 0;
int reverserControl = 0;

/*---------------------------------------------------------------------------*/
/*                                Intake Function                           */

void intakeGrabe()
{
  intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
}

void intakeReverse()
{
  intake.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
}

void stopIntake()
{
  intake.stop();
}

/*---------------------------------------------------------------------------*/
/*                            Autonomous Functions                           */

// elementary autonoumous movements

//========================================================================================
//========================================================================================

// move toward desired location using drivetrain initialized with hyperparametrs through testing
void moveRobot(float distance, int timout = 5)
{

  Drivetrain.setTimeout(timout, sec);
  Drivetrain.driveFor(-distance, mm);
}

// function to make robot turn toward angle in degree, use a hyperparameter that has been found by testing
// this funtion use only one set (Right or Left) of motors to turn the robot
// Good for not making the ball spli but bad for speed, very slow
// the reverse parameter is set to false to make the robot turn while going forward, true while going backward
// TO DO: add a timout similar to the moveForward timeout
void turnRobotToAngle1D(float angle, bool reverse = false)
{
  if (!reverse)
  {
    if (angle < 0)
    {
      RightDriveSmart.spinFor(TURN_ANGLE_MOTOR_RATIO * angle, deg);
    }
    else
    {
      LeftDriveSmart.spinFor(-TURN_ANGLE_MOTOR_RATIO * angle, deg);
    }
  }
  else
  {
    if (angle < 0)
    {
      LeftDriveSmart.spinFor(-TURN_ANGLE_MOTOR_RATIO * angle, deg);
    }
    else
    {
      RightDriveSmart.spinFor(TURN_ANGLE_MOTOR_RATIO * angle, deg);
    }
  }
}

// function tha uses both the righ and left mototors to make a turn
// good speed but bad ball control

// TO DO: add a timout similar to the moveForward timeout
void turnRobotToAngle2D(float angle, bool reverse = false)
{
  if (angle < 0)
  {
    RightDriveSmart.spinFor(TURN_ANGLE_MOTOR_RATIO / 2 * angle, deg);
    LeftDriveSmart.spinFor(-TURN_ANGLE_MOTOR_RATIO / 2 * angle, deg);
  }
  else
  {
    LeftDriveSmart.spinFor(-TURN_ANGLE_MOTOR_RATIO / 2 * angle, deg);
    RightDriveSmart.spinFor(TURN_ANGLE_MOTOR_RATIO / 2 * angle, deg);
  }
}

// function to both move toward and turn an angle
void moveAndTurn(float distance, float angle)
{
  moveRobot(distance);
  turnRobotToAngle2D(angle);
}

// function to activate the shield
void shieldControlFunction()
{
  shieldMotor.setVelocity(100, pct);
  if (shiledUp == 0)
  {
    shieldMotor.spinFor(directionType::fwd, 100, rotationUnits::deg);
    shiledUp = 1;
  }

  else if (shiledUp == 1)
  {
    shieldMotor.spinFor(directionType::fwd, -100, rotationUnits::deg);
    shiledUp = 0;
    shieldMotor.setBrake(brakeType::hold);
  }
}

float get_speed_direction(const char *side)
{
  if (reverserControl)
  {
    if (strcmp(side, "right"))
    {
      return -Controller2.Axis3.position() + Controller2.Axis1.position();
    }

    if (strcmp(side, "left"))
    {
      return -Controller2.Axis3.position() - Controller2.Axis1.position();
    }
  }

  if (strcmp(side, "right"))
  {
    return -Controller2.Axis3.position() - Controller2.Axis1.position();
  }

  if (strcmp(side, "left"))
  {
    return Controller2.Axis1.position() - Controller2.Axis3.position();
  }

  return 0;
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

  // forward 600
  moveRobot(600);
  // turn 90 with reverse
  turnRobotToAngle1D(60, false);
  // move forward 100
  Drivetrain.setDriveVelocity(150, pct);
  moveRobot(500, 2);

  turnRobotToAngle1D(30);
  moveRobot(50, 2);
  Drivetrain.setDriveVelocity(70, pct);
  moveRobot(-250, 2);

  Drivetrain.setDriveVelocity(200, pct);
  moveRobot(300, 2);

  Drivetrain.setDriveVelocity(100, pct);
  moveRobot(-120);
  turnRobotToAngle1D(-65, true);

  moveRobot(-350);
  turnRobotToAngle1D(-45, true);
  turnRobotToAngle1D(6);
  moveRobot(-3700);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own gprobot specific commands here.   */
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
    // ........................................................................

    RightDriveSmart.spin(vex::directionType::fwd,
                         get_speed_direction("right"),
                         vex::velocityUnits::pct);
    LeftDriveSmart.spin(vex::directionType::fwd, get_speed_direction("left"),
                        vex::velocityUnits::pct);

    // Field dimensions: 3657.6mm x 3657.6mm

    /*---------------------------------------------------------------------------*/
    /*                             Flyweel Control                               */

    Controller2.ButtonX.pressed([]()
                                { shieldControlFunction(); });

    if (Controller2.ButtonR1.pressing())
    {
      intakeGrabe();
    }
    else if (Controller2.ButtonL1.pressing())
    {
      intakeReverse();
    }
    else
    {
      stopIntake();
    }

    /* track_location();

    current_motor_angle_left = leftMotorA.position(deg);
    current_motor_angle_right = rightMotorA.position(deg); */

    /*---------------------------------------------------------------------------*/
    /*                             Test Autonomous                             */
    if (Controller2.ButtonB.pressing())
    {
      autonomous();
    }

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

/****************************************************************************/
/*                             DO NOT DELETE                                */
/*                                                                          */
/*                              GPS Testing                                 */
/*                                                                          */
/*                                                                          */
/****************************************************************************/

/*
void drive_to_with_gps(double x_target, double y_target) {
  double x_distance_to_drive = x_target - gps_sensor.xPosition(mm);

  double y_distance_to_drive = y_target - gps_sensor.yPosition(mm);

  if (x_distance_to_drive == 0 && y_distance_to_drive == 0) {
    return;
  }

  double magnitude = sqrt(x_distance_to_drive * x_distance_to_drive + y_distance_to_drive * y_distance_to_drive);

  double angle = std::asin((std::sin(M_PI / 2.0) * x_distance_to_drive) / magnitude) * 180.0 / M_PI;

  if (y_distance_to_drive < 0) {
    angle = 180 - angle;
  }

  Drivetrain.turnFor(angle, degrees, true);
  Drivetrain.driveFor(magnitude, mm, true);
}




void gps_drive_test() {
  Drivetrain.setDriveVelocity(20, pct);
  Drivetrain.setTurnVelocity(20, pct);

  double x_target = 1.3;
  double y_target = 0.5;

  while (gps_sensor.xPosition(mm) != x_target || gps_sensor.yPosition(mm) != y_target) {
    if (gps_sensor.xPosition(mm) > x_target-0.05 || gps_sensor.xPosition(mm) < x_target+0.05) {
      return;
    }

    if (gps_sensor.yPosition(mm) > y_target-0.05 || gps_sensor.yPosition(mm) < y_target+0.05) {
      return;
    }

    drive_to_with_gps(x_target, y_target);
  }

}
*/

// if (Controller1.ButtonB.pressing()) {
//   gps_drive_test();
// }

/****************************************************************************/
/*                                                                          */
/*                                                                          */
/*                                    END                                   */
/*                                                                          */
/*                                                                          */
/****************************************************************************/

// unused functions ==============================================================================
int current_motor_angle_left = 0;
int current_motor_angle_right = 0;
float d = 0;
float rw = 385 / 2;
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

  if (std::abs(dtheta) <= 0.01)
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
