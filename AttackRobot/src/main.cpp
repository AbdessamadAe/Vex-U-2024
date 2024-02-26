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
float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER  * M_PI;
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
motor shieldMotorA = motor(PORT11, ratio18_1, false);
motor shieldMotorB = motor(PORT3, ratio18_1, true);
motor_group shieldMotor = motor_group(shieldMotorA, shieldMotorB);

motor wingmotorA = motor(PORT15, ratio18_1, false);
motor wingmotorB = motor(PORT16, ratio18_1, true);
motor_group wingmotors = motor_group(wingmotorA, wingmotorB);


int shiledUp = 0;
int WingsExtended = 0;
bool reverserControl = false;
time_t controllerStartTimer = time(NULL);


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

// function to activate the shield
void shieldControlFunction()
{
  if (shiledUp == 0)
  {
    shieldMotor.setVelocity(200, pct);
    shieldMotor.spinFor(directionType::fwd, 120, rotationUnits::deg);
    shiledUp = 1;
  }

  else if (shiledUp == 1)
  { 
    shieldMotor.setVelocity(10, pct);
    shieldMotor.spinFor(directionType::fwd, -120, rotationUnits::deg);
    shiledUp = 0;
    shieldMotorA.setBrake(brakeType::hold);
    shieldMotorB.setBrake(brakeType::hold);
  }
}

void wingsFunction()
{
  
  if (WingsExtended == 0)
  {
    wingmotors.setVelocity(20, pct);
    wingmotors.spinFor(directionType::fwd, 145, rotationUnits::deg);
    WingsExtended = 1;
  }

  else if (WingsExtended == 1)
  {
    wingmotors.setVelocity(20, pct);
    wingmotors.spinFor(directionType::fwd, -145, rotationUnits::deg);
    WingsExtended = 0;
    wingmotorA.setBrake(brakeType::hold);
    wingmotorB.setBrake(brakeType::hold);
  }
}

float get_speed_direction(const char *side)
{
  if (reverserControl)
  {
    if (strcmp(side, "right"))
    {
      return Controller2.Axis3.position() - Controller2.Axis1.position();
    }

    if (strcmp(side, "left"))
    {
      return Controller2.Axis3.position() + Controller2.Axis1.position();
    }
  }

  if (strcmp(side, "right"))
  {
    return -Controller2.Axis3.position() - Controller2.Axis1.position();
  }

  if (strcmp(side, "left"))
  {
    return -Controller2.Axis3.position() + Controller2.Axis1.position();
  }

  return 0;
}

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

  LeftDriveSmart.setStopping(brakeType::hold);
  RightDriveSmart.setStopping(brakeType::hold);

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


//mm to degrees
float mm_to_deg(int distance_mm){
    float rev = distance_mm / WHEEL_CIRCUMFERENCE;
    return  rev * 360;
}

void moveForward(int distance_mm, int speed=200){

    RightDriveSmart.resetPosition();
    LeftDriveSmart.resetPosition();

    float dist_deg = -mm_to_deg(distance_mm) * 0.8; // need to be fine tuned

    RightDriveSmart.spinTo(dist_deg, deg, speed, rpm, false);
    LeftDriveSmart.spinTo(dist_deg, deg, speed, rpm, true);
}

void turn_angle_2D(int angle, int speed=200){
    RightDriveSmart.resetPosition();
    LeftDriveSmart.resetPosition();

      float deg_angle = -angle * 2.62;
      LeftDriveSmart.spinTo(deg_angle,  deg, speed, rpm, false);
      RightDriveSmart.spinTo(-deg_angle,  deg, speed, rpm);
}

void turn_angle_1D(int angle, int speed=200, bool reverse=false){
    RightDriveSmart.resetPosition();
    LeftDriveSmart.resetPosition();

      float deg_angle = -angle * 2.62;

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




void autonomous(void)
{
  moveForward(400, 50);
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

  // User control code here, inside the loop
  while (1)
  {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    RightDriveSmart.spin(vex::directionType::fwd,
                         get_speed_direction("right"),
                         vex::velocityUnits::pct);
    LeftDriveSmart.spin(vex::directionType::fwd, get_speed_direction("left"),
                        vex::velocityUnits::pct);


    Controller2.ButtonX.pressed([]()
                                { shieldControlFunction(); });

    Controller2.ButtonA.pressed([]()
                                { wingsFunction(); });

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

    if (Controller2.ButtonR2.pressing())
    {
      switch_control_direction(&controllerStartTimer);
    }

    if (Controller2.ButtonL2.pressing()){
      autonomous();
    }

    wait(20, msec);
  }
}


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

// if (Controller2.ButtonB.pressing()) {
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
