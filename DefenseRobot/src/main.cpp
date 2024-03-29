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

#include "objectDtection.h"
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
float GEAR_RATIO = 2.9; // 84/48;
float WHEEL_DIAMETER = 101.6;
float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
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
vision visionSensor = vision(PORT7);
triport Threewireport = triport(PORT22);
limit switch_sensor = limit(Threewireport.A);
inertial inertial_sensor = inertial(PORT16);

motor FlywheelA = motor(PORT4, ratio18_1, false);
motor_group Flywheel = motor_group(FlywheelA);
motor Armmotor = motor(PORT8, ratio18_1, false);

motor wingmotorL = motor(PORT11, ratio18_1, true);
motor wingmotorR = motor(PORT7, ratio18_1, false);
motor_group wingmotor = motor_group(wingmotorL, wingmotorR);


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
int armUp = 0;
int flywheelOn = 0;
int WingLExtended = 0;
int WingRExtended = 0;

int auton = 0;


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

  RightDriveSmart.setStopping(hold);
  LeftDriveSmart.setStopping(hold);
  
  return;
}

void armControlFunction(double i)
{
  Armmotor.setVelocity(100, pct);
  if (armUp == 0)
  {
    Armmotor.spinFor(directionType::fwd, i * 360, rotationUnits::deg);
    armUp = 1;
    Armmotor.setBrake(brakeType::hold);

  }

  else if (armUp == 1)
  {
    Armmotor.setTimeout(1000, msec);
    Armmotor.spinFor(directionType::fwd, -i * 360, rotationUnits::deg);
    armUp = 0;
    Armmotor.setBrake(brakeType::coast);
  }
}

void flywheel(int speed)
{
  if (flywheelOn == 0)
  {
    FlywheelA.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
    flywheelOn = 1;
  }
  else if (flywheelOn == 1)
  {
    FlywheelA.stop(vex::brakeType::brake);
    flywheelOn = 0;
  }
}

/*---------------------------Wing Function-------------------------------*/

void wingRFunction()
{
  
  if (WingRExtended == 0)
  {
    wingmotorR.setVelocity(50, pct);
    wingmotorR.spinFor(directionType::fwd, 100, rotationUnits::deg);
    WingRExtended = 1;
  }

  else if (WingRExtended == 1)
  {
    wingmotorR.setVelocity(30, pct);
    wingmotorR.spinFor(directionType::fwd, -100, rotationUnits::deg);
    WingRExtended = 0;
    wingmotorR.setBrake(brakeType::hold);
  }
}

void wingLFunction()
{
  
  if (WingLExtended == 0)
  {
    wingmotorL.setVelocity(50, pct);
    wingmotorL.spinFor(directionType::fwd, 100, rotationUnits::deg);
    WingLExtended = 1;
  }

  else if (WingLExtended == 1)
  {
    wingmotorL.setVelocity(30, pct);
    wingmotorL.spinFor(directionType::fwd, -100, rotationUnits::deg);
    WingLExtended = 0;
    wingmotorL.setBrake(brakeType::hold);
  }
}

void wingFunction(){
  wingRFunction();
  wingLFunction();
}

void ButtonAwingFunction(){
  if (reverserControl){
    wingLFunction();
  }
  else{
    wingRFunction();
  }
}

void ButtonYwingFunction(){
  if (reverserControl){
    wingRFunction();
  }
  else{
    wingLFunction();
  }
}

//mm to degrees
float mm_to_deg(int distance_mm){
    float rev = distance_mm / WHEEL_CIRCUMFERENCE;
    return  rev * 360;
}

void moveForward(int distance_mm, int speed=200){

    RightDriveSmart.resetPosition();
    LeftDriveSmart.resetPosition();

    float dist_deg = mm_to_deg(distance_mm);

    RightDriveSmart.spinTo(dist_deg, deg, speed, rpm, false);
    LeftDriveSmart.spinTo(dist_deg, deg, speed, rpm, true);
}

void moveInCurve(double right, double left, int r_speed, int l_speed){

    RightDriveSmart.resetPosition();
    LeftDriveSmart.resetPosition();

    RightDriveSmart.spinTo(right, deg, r_speed, rpm, false);
    LeftDriveSmart.spinTo(left, deg, l_speed, rpm, true);
}

void turn_angle_2D(int angle, int speed=200){
    RightDriveSmart.resetPosition();
    LeftDriveSmart.resetPosition();

      float deg_angle = angle * 2.57;
      LeftDriveSmart.spinTo(deg_angle,  deg, speed, rpm, false);
      RightDriveSmart.spinTo(-deg_angle,  deg, speed, rpm);
}

void turn_angle_1D(int angle, int speed=200, bool reverse=false){
    RightDriveSmart.resetPosition();
    LeftDriveSmart.resetPosition();

      float deg_angle = angle * 2.57;

      if(reverse){
          if (angle > 0){
            LeftDriveSmart.spinTo(-deg_angle*2,  deg, speed, rpm);
          }
          else {
            RightDriveSmart.spinTo(-deg_angle*2,  deg, speed, rpm);
          }
      }
      else {
          if (angle > 0){
            LeftDriveSmart.spinTo(deg_angle*2,  deg, speed, rpm);
          }
          else {
            RightDriveSmart.spinTo(deg_angle*2,  deg, speed, rpm);
          }
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

  //for autonomous to run only one time
  auton = 1;

  //part 1
  wait(0.2, sec);
  moveForward(990, 90);
  wait(0.2, sec);
  turn_angle_2D(90, 70);
  wait(0.2, sec);
  wingFunction();
  wait(0.2, sec);
  moveForward(850, 200);
  wait(0.2, sec);
  wingFunction();
  wait(.2, sec);
  moveForward(-400, 200);
  wait(.2, sec);
  moveForward(520, 150);
  wait(.2, sec);
  moveForward(-300, 200);
  wait(.2, sec);
  moveForward(340, 100);

  //part 2
  moveForward(-740, 150);
  wait(0.2, sec);
  turn_angle_2D(90, 150);
  wait(0.2, sec);
  moveForward(1250, 150);
  wait(0.2, sec);
  turn_angle_2D(-85, 150);
  wait(0.2, sec);
  moveForward(900, 150);
  wait(0.2, sec);
  moveForward(-900, 150);

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
  wingmotorL.setBrake(brakeType::hold);
  wingmotorR.setBrake(brakeType::hold);

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
    if (Controller1.ButtonUp.pressing() && auton == 0)
    {
      autonomous();
    }

    if (Controller1.ButtonR1.pressing())
    {
      switch_control_direction(&controllerStartTimer);
    }
    Controller1.ButtonB.pressed([]()
                                { armControlFunction(3.6); });
    
    Controller1.ButtonL1.pressed([]()
                                { armControlFunction(2); });                            
    Controller1.ButtonX.pressed([]()
                                { flywheel(100); });
    Controller1.ButtonA.pressed([]()
                                { ButtonAwingFunction(); });
    Controller1.ButtonY.pressed([]()
                                { ButtonYwingFunction(); });

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
    screen.printAt(10, 20, "RightDrivesmart: %f", rightFrontMotor.position(deg));
    screen.printAt(10, 50, "LeftDrivesmart: %f", leftFrontMotor.position(deg));

    if (Controller1.ButtonL2.pressing())
    {
      RightDriveSmart.resetPosition();
      LeftDriveSmart.resetPosition();
      Brain.Screen.clearScreen();
    }

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
    /* if (Controller1.ButtonL1.pressing())
    {
      return;
    } */

    wait(20, msec);
  }
}

// function for automatically staring the robot toward a detected green triball
// given the visionSensor 
//object that took the snapshot
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
      return -Controller1.Axis3.position() + 0.7*Controller1.Axis1.position();
    }

    if (strcmp(side, "left"))
    {
      return -Controller1.Axis3.position() - 0.7*Controller1.Axis1.position();
    }
  }

  if (strcmp(side, "right"))
  {
    return Controller1.Axis3.position() + 0.7*Controller1.Axis1.position();
  }

  if (strcmp(side, "left"))
  {
    return Controller1.Axis3.position() - 0.7*Controller1.Axis1.position();
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