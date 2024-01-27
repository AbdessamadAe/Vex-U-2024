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
int GEAR_RATIO = 3; //84/48;
float WHEEL_DIAMETER = 101.6;
float WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER/2) * M_PI;
float TURN_ANGLE_MOTOR_RATIO = 5.4;

// define your global instances of motors and other devices here
motor leftMotorA = motor(PORT10, ratio18_1, false);
motor leftMotorB = motor(PORT20, ratio18_1, false);
motor rightMotorA = motor(PORT1, ratio18_1, true);
motor rightMotorB = motor(PORT11, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, WHEEL_CIRCUMFERENCE, 390, 315, mm, GEAR_RATIO);
controller Controller2 = controller(primary);
motor FlywheelA = motor(PORT7, ratio18_1, false);
motor FlywheelB = motor(PORT8, ratio18_1, true);
motor_group Flywheel = motor_group(FlywheelA, FlywheelB);
motor Armmotor = motor(PORT9, ratio18_1, false);

int armUp = 0;

/*---------------------------------------------------------------------------*/
/*                                Flywheel Function                           */


/*---------------------------------------------------------------------------*/
/*                            Autonomous Functions                           */




//elementary autonoumous movements 

//========================================================================================
//========================================================================================

//move toward desired location using drivetrain initialized with hyperparametrs through testing
void moveRobot(float distance, int timout=0)
{
  if(timout != 0){
    Drivetrain.setTimeout(timout, sec);
  }
  Drivetrain.driveFor(-distance, mm);
}


//function to make robot turn toward angle in degree, use a hyperparameter that has been found by testing 
//this funtion use only one set (Right or Left) of motors to turn the robot 
//Good for not making the ball spli but bad for speed, very slow 
//the reverse parameter is set to false to make the robot turn while going forward, true while going backward
//TO DO: add a timout similar to the moveForward timeout
void turnRobotToAngle1D(float angle, bool reverse=false)
{
  if(!reverse){
    if (angle < 0)
    {
      RightDriveSmart.spinFor(TURN_ANGLE_MOTOR_RATIO * angle, deg);
    }
    else
    {
      LeftDriveSmart.spinFor(-TURN_ANGLE_MOTOR_RATIO * angle, deg);
    }
  }
  else{
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


//function tha uses both the righ and left mototors to make a turn
//good speed but bad ball control 

//TO DO: add a timout similar to the moveForward timeout
void turnRobotToAngle2D(float angle, bool reverse=false)
{
  if (angle < 0)
  {
    RightDriveSmart.spinFor(TURN_ANGLE_MOTOR_RATIO/2 * angle, deg);
    LeftDriveSmart.spinFor(-TURN_ANGLE_MOTOR_RATIO/2 * angle, deg);
  }
  else
  {
    LeftDriveSmart.spinFor(-TURN_ANGLE_MOTOR_RATIO/2 * angle, deg);
    RightDriveSmart.spinFor(TURN_ANGLE_MOTOR_RATIO/2 * angle, deg);
  }
}


//function to both move toward and turn an angle
void moveAndTurn(float distance, float angle)
{
  moveRobot(distance);
  turnRobotToAngle2D(angle);
}


//function to activate the arm and start the flywheel rotation, take speed of flywheel as argument
void flywheel(int speed)
{
  if (armUp == 0)
  {
    Armmotor.spinFor(directionType::fwd, 3 * 360, rotationUnits::deg);
    armUp = 1;
  }
  Flywheel.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
}


//function to stop the flywheel and take the arm down
void Stopflywheel()
{
  Flywheel.stop();
  if (armUp == 1)
  {
    Armmotor.spinFor(directionType::fwd, -3 * 360, rotationUnits::deg);
    armUp = 0;
  }

  Armmotor.setBrake(brakeType::hold);
}

//===============================================================================================
//================================================================================================




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

  //forward 600
      moveRobot(600);
      //turn 90 with reverse
      turnRobotToAngle1D(60, false);
      //move forward 100
      moveRobot(500, 2);

      turnRobotToAngle1D(30);
      moveRobot(50, 2);
      moveRobot(-250, 2);
      Drivetrain.setDriveVelocity(300, pct);
      moveRobot(300, 2);
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
    RightDriveSmart.spin(vex::directionType::fwd, -Controller2.Axis2.position() + Controller2.Axis4.position(), pct);
    LeftDriveSmart.spin(vex::directionType::fwd, -Controller2.Axis2.position() - Controller2.Axis4.position(), pct);
    // ........................................................................

    // Field dimensions: 3657.6mm x 3657.6mm

    /*---------------------------------------------------------------------------*/
    /*                             Flyweel Control                               */

    Controller2.ButtonX.pressed([]()
                                { flywheel(100); });
    Controller2.ButtonY.pressed([]()
                                { Stopflywheel(); });

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

















//unused functions ==============================================================================
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
