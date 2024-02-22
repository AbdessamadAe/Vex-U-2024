/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <cmath>
#include <ctime>

// #include "objectDtection.h"
#include "vex.h"

using namespace vex;

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                             Global Instances                              */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
// A global instance of competition
competition Competition;
int GEAR_RATIO = 2.9; // 84/48;
float WHEEL_DIAMETER = 101.6;
float WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER / 2) * M_PI;
float TURN_ANGLE_MOTOR_RATIO = 5.4;


// define your global instances of motors and other devices here
motor leftFrontMotor = motor(PORT3, ratio18_1, false);
motor leftBackMotor = motor(PORT12, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftFrontMotor, leftBackMotor);
motor rightFrontMotor = motor(PORT10, ratio18_1, true);
motor rightBackMotor = motor(PORT13, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightFrontMotor, rightBackMotor);

//track width = 260, wheel base = 170
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, WHEEL_CIRCUMFERENCE, 260, 170, mm, GEAR_RATIO);

controller Controller1 = controller(primary);
brain::lcd screen = vex::brain::lcd();
vision visionSensor = vision(PORT10);
triport Threewireport = triport(PORT22);
limit switch_sensor = limit(Threewireport.A);
inertial inertial_sensor = inertial(PORT16);

motor FlywheelA = motor(PORT4, ratio18_1, false);
motor FlywheelB = motor(PORT5, ratio18_1, true);
motor_group Flywheel = motor_group(FlywheelA, FlywheelB);
motor Armmotor = motor(PORT8, ratio18_1, false);

motor wingmotor = motor(PORT11, ratio18_1, false);

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                             Global Variables                              */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = false;

// custom global variables
double obstacle_distance = 100000;
bool reverserControl = false;
time_t controllerStartTimer = time(NULL);
int current_motor_angle_left = 0;
int current_motor_angle_right = 0;
float d = 0;
float rw = 140;
float wheeldiam = 101.6;
float dtheta = 0;
int stop = 0;
int armUp = 0;
int flywheelOn = 0;
int WingExtended = 0;

struct
{
  int X = 0;
  int Y = 0;
  float theta = 0.0;
} position;

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                     Custom Function Prototypes                            */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
void face_angle_smooth(float target_angle = 50.0, float acceptable_error = 5);
float get_speed_direction(const char *side);
void switch_control_direction(time_t *controllerStartTimer);
void auto_face_greentriball(vision visionSensor);

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
  // DO NOT REMOVE! Initializing Robot Configuration.
  vexcodeInit();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  return;
}

void armControlFunction()
{
  Armmotor.setVelocity(100, pct);
  if (armUp == 0)
  {
    Armmotor.spinFor(directionType::fwd, 3.5 * 360, rotationUnits::deg);
    armUp = 1;
  }

  else if (armUp == 1)
  {
    Armmotor.spinFor(directionType::fwd, -3.5 * 360, rotationUnits::deg);
    armUp = 0;
    Armmotor.setBrake(brakeType::hold);
  }
}

void flywheel(int speed)
{
  if (flywheelOn == 0)
  {
    Flywheel.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
    flywheelOn = 1;
  }
  else if (flywheelOn == 1)
  {
    Flywheel.stop(vex::brakeType::brake);
    flywheelOn = 0;
  }
}

/*---------------------------Wing Function-------------------------------*/

void wingFunction()
{
  
  if (WingExtended == 0)
  {
    wingmotor.setVelocity(50, pct);
    wingmotor.spinFor(directionType::fwd, 210, rotationUnits::deg);
    WingExtended = 1;
  }

  else if (WingExtended == 1)
  {
    wingmotor.setVelocity(20, pct);
    wingmotor.spinFor(directionType::fwd, -210, rotationUnits::deg);
    WingExtended = 0;
    wingmotor.setBrake(brakeType::hold);
  }
}


//autonomous functions
// move toward desired location using drivetrain initialized with hyperparametrs through testing
void moveRobot(float distance, int timout = 10)
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

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                             Autonomous Phase                              */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
void autonomous(void)
{
  
  moveRobot(400);
  // drivetrain code to be tested
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                            User Control Phase                             */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
void usercontrol(void)
{
  LeftDriveSmart.setStopping(brakeType::hold);
  RightDriveSmart.setStopping(brakeType::hold);
  Drivetrain.setDriveVelocity(50, pct);
  Drivetrain.setTurnVelocity(25, pct);
  inertial_sensor.calibrate();

  while (inertial_sensor.isCalibrating())
  {
    wait(50, msec);
  }

  rightFrontMotor.setPosition(0, deg);
  leftFrontMotor.setPosition(0, deg);

  /***************************************************************************/
  /*        Build functionalities for your Joystick inside the loop          */
  /***************************************************************************/
  while (1)
  {
    /*************************************************************************/
    /*                         Quick Buttons Set Up                          */
    /*************************************************************************/
    if (Controller1.ButtonR1.pressing())
    {
      autonomous();
    }

    if (Controller1.ButtonR2.pressing())
    {
      switch_control_direction(&controllerStartTimer);
    }


    Controller1.ButtonX.pressed([]()
                                { armControlFunction(); });
    Controller1.ButtonY.pressed([]()
                                { flywheel(100); });
    Controller1.ButtonA.pressed([]()
                                { wingFunction(); });

    /*************************************************************************/
    /*                              Drivetrain                               */
    /*************************************************************************/
    obstacle_distance = 100000;

    if (obstacle_distance > 150 || Controller1.Axis1.position() < 0)
    {
      RightDriveSmart.spin(vex::directionType::fwd,
                           get_speed_direction("right"),
                           vex::velocityUnits::pct);
      LeftDriveSmart.spin(vex::directionType::fwd, get_speed_direction("left"),
                          vex::velocityUnits::pct);
    }
    else
    {
      RightDriveSmart.stop(vex::brakeType::brake);
      LeftDriveSmart.stop(vex::brakeType::brake);
    }

    /*************************************************************************/
    /*         Tracking the Motors Angle && the Heading of the Robot         */
    /*************************************************************************/
    current_motor_angle_left = leftFrontMotor.position(deg);
    current_motor_angle_right = rightFrontMotor.position(deg);
    position.theta = inertial_sensor.heading() * 180 / M_PI;

    /*************************************************************************/
    /*                            Temp Code                                  */
    /*************************************************************************/
    screen.printAt(10, 10, "Dist: %f", obstacle_distance);
    screen.printAt(10, 60, "Reverse Control: %d", reverserControl);
    screen.printAt(10, 90, "Inertial Sensor heading: %f",
                   inertial_sensor.heading(degrees));

    stop = 1;

    /******************************* END ************************************/
    wait(20, msec); // Sleep the task to prevent wasted resources.
  }
} // the end of the user control mode

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                Set Up Competition Functions & Callbacks                   */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting using an infinite loop.
  while (true)
  {
    wait(100, msec);
  }
} // End of the Main function











/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                       Custom Function Definitions                         */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
void face_angle_smooth(float target_angle, float acceptable_error)
{
  float current_angle = inertial_sensor.heading();
  float error_angle = current_angle - target_angle;
  float motor_speed;

  while (std::abs(error_angle) > acceptable_error)
  {
    if (error_angle > 0)
    {
      if (error_angle <= 180)
      {
        // turn left
        motor_speed = error_angle * 0.5;
      }
      else
      {
        // turn right
        motor_speed = -(360 - error_angle) * 0.5;
      }
    }
    else
    {
      if (-error_angle <= 180)
      {
        // turn right
        motor_speed = error_angle * 0.5;
      }
      else
      {
        motor_speed = (360 + error_angle) * 0.5;
      }
    }

    RightDriveSmart.spin(fwd, motor_speed, pct);
    LeftDriveSmart.spin(fwd, -motor_speed, pct);

    current_angle = inertial_sensor.heading();
    error_angle = current_angle - target_angle;
    // screen.printAt(10, 180, "Turning smoothly ....");

    // break the automated turning in case the robot stuck
    if (Controller1.ButtonL1.pressing())
    {
      return;
    }

    wait(20, msec);
  }
}

// function for automatically staring the robot toward a detected green triball
// given the visionSensor object that took the snapshot
// It will also move the robot toward the triball until collision is detected
// with the front switch sensor take into account the location of the camera and
// a margin error (optional)
void auto_face_greentriball(vision visionSensor)
{
  int error_margin = 50, camera_x = 158;

  if (visionSensor.largestObject.exists)
  {
    int triball_x = visionSensor.largestObject.centerX;
    if (triball_x <= (camera_x - error_margin))
    {
      // turn left
      float motor_speed = 25 * triball_x / camera_x;
      RightDriveSmart.spin(vex::directionType::fwd, motor_speed, pct);
      LeftDriveSmart.spin(vex::directionType::rev, motor_speed, pct);
    }
    else if (triball_x >= (camera_x + error_margin))
    {
      // turn right
      float motor_speed = 25 * (1 - camera_x / triball_x);
      RightDriveSmart.spin(vex::directionType::rev, motor_speed, pct);
      LeftDriveSmart.spin(vex::directionType::fwd, motor_speed, pct);
    }
    else if (!switch_sensor.pressing())
    {
      RightDriveSmart.spin(fwd, 25, pct);
      LeftDriveSmart.spin(fwd, 25, pct);
    }
    else
    {
      // break
      RightDriveSmart.stop(vex::brakeType::brake);
      LeftDriveSmart.stop(vex::brakeType::brake);
    }
  }
  return;
}

// function to get the speed for the rotation of the motors depending on wether
// the controls are reversed or not reversing the controls specifically the
// Axis1 is done by pressing R2 This can be useful for easier control of the
// robots when its rotated 180 degrees
float get_speed_direction(const char *side)
{
  if (reverserControl)
  {
    if (strcmp(side, "right"))
    {
      return -Controller1.Axis3.position() - Controller1.Axis1.position();
    }

    if (strcmp(side, "left"))
    {
      return -Controller1.Axis3.position() + Controller1.Axis1.position();
    }
  }

  if (strcmp(side, "right"))
  {
    return -Controller1.Axis3.position() + Controller1.Axis1.position();
  }

  if (strcmp(side, "left"))
  {
    return -Controller1.Axis3.position() - Controller1.Axis1.position();
  }

  return 0;
}

// function to switch the controller flag after a 1 second cooldown
void switch_control_direction(time_t *controllerStartTimer)
{
  time_t currentTime = time(NULL);
  if (difftime(currentTime, *controllerStartTimer) < 1)
  {
    return;
  }
  *controllerStartTimer = time(NULL);

  if (reverserControl)
  {
    reverserControl = false;
  }
  else
  {
    reverserControl = true;
  }
}