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
const double GEAR_RATIO = 2/7; // 24/84;
const float WHEEL_DIAMETER = 101.6;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER  * M_PI;
const float TURN_ANGLE_MOTOR_RATIO = 5.4;
const double MAX_DRIVETRAIN_VELOCITY_RPM = 130;
const double MAX_INTAKE_VELOCITY_RPM = 170;

// define your global instances of motors and other devices here
controller Controller2 = controller(primary);

// Drivetrain
motor leftFrontMotor = motor(PORT1, ratio18_1, false);
motor leftBackMotor = motor(PORT11, ratio18_1, false);
motor rightFrontMotor = motor(PORT19, ratio18_1, true);
motor rightBackMotor = motor(PORT14, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(leftFrontMotor, leftBackMotor);
motor_group RightDriveSmart = motor_group(rightFrontMotor, rightBackMotor);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, WHEEL_CIRCUMFERENCE, 390, 315, mm, GEAR_RATIO);

// Intake
motor intakeMotorA = motor(PORT10, ratio18_1, false);
motor intakeMotorB = motor(PORT20, ratio18_1, true);
motor_group intake = motor_group(intakeMotorA, intakeMotorB);

// Wing
motor rightWingMotor = motor(PORT15, ratio18_1, false);
motor leftWingMotor = motor(PORT16, ratio18_1, true);
motor_group wingMotors = motor_group(rightWingMotor, leftWingMotor);

bool reverserControl = false;
time_t controllerStartTimer = time(NULL);


/*---------------------------------------------------------------------------*/
/*                                Intake Function                           */
/*---------------------------------------------------------------------------*/
void moveIntakeToInside() {
  intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
}

void moveIntakeToOutside() {
  intake.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
}

void stopIntake() {
  intake.stop();
}

/*---------------------------------------------------------------------------*/
/*                            Autonomous Functions                           */
/*---------------------------------------------------------------------------*/

// int shieldUp = 0;

// void openAndCloseShield() {
//   if (shieldUp == 0) {
//     shieldMotor.setVelocity(50, rpm);
//     shieldMotor.spinFor(directionType::fwd, 250, rotationUnits::deg);
//     shieldUp = 1;
//   }

//   else if (shieldUp == 1) { 
//     shieldMotor.setVelocity(10, rpm);
//     shieldMotor.spinFor(directionType::fwd, -250, rotationUnits::deg);
//     shieldUp = 0;
//     shieldMotorA.setBrake(brakeType::hold);
//     shieldMotorB.setBrake(brakeType::hold);
//   }
// }


int WingsExtended = 0;

void wingsFunction() {
  if (WingsExtended == 0) {
    wingMotors.setVelocity(20, pct);
    wingMotors.spinFor(directionType::fwd, 145, rotationUnits::deg);
    WingsExtended = 1;
  }

  else if (WingsExtended == 1) {
    wingMotors.setVelocity(20, pct);
    wingMotors.spinFor(directionType::fwd, -145, rotationUnits::deg);
    WingsExtended = 0;
    rightWingMotor.setBrake(brakeType::hold);
    leftWingMotor.setBrake(brakeType::hold);
  }
}

float get_speed_direction(const char *side) {
  if (reverserControl) {
    if (strcmp(side, "right")) {
      return Controller2.Axis3.position(pct) - Controller2.Axis1.position(pct);
    }

    if (strcmp(side, "left")) {
      return Controller2.Axis3.position(pct) + Controller2.Axis1.position(pct);
    }
  }

  if (strcmp(side, "right")) {
    return -Controller2.Axis3.position(pct) - Controller2.Axis1.position(pct);
  }

  if (strcmp(side, "left")) {
    return -Controller2.Axis3.position(pct) + Controller2.Axis1.position(pct);
  }

  return 0;
}

void switch_control_direction(time_t *controllerStartTimer) {
  time_t currentTime = time(NULL);
  if (difftime(currentTime, *controllerStartTimer) < 1) {
    return;
  }
  *controllerStartTimer = time(NULL);

  if (reverserControl) {
    reverserControl = false;
  } else {
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

void pre_auton(void) {
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
float mm_to_deg(int distance_mm) {
    float rev = distance_mm / WHEEL_CIRCUMFERENCE;
    return  rev * 360;
}

void moveForward(int distance_mm, int speed=200) {
    RightDriveSmart.resetPosition();
    LeftDriveSmart.resetPosition();

    float dist_deg = -mm_to_deg(distance_mm) * 0.67; // need to be fine tuned

    RightDriveSmart.spinTo(dist_deg, deg, speed, rpm, false);
    LeftDriveSmart.spinTo(dist_deg, deg, speed, rpm, true);
}

void turn_angle_2D(int angle, int speed=200) {
    RightDriveSmart.resetPosition();
    LeftDriveSmart.resetPosition();

      float deg_angle = -angle * 2.62;
      LeftDriveSmart.spinTo(deg_angle,  deg, speed, rpm, false);
      RightDriveSmart.spinTo(-deg_angle,  deg, speed, rpm);
}

void turn_angle_1D(int angle, int speed=200, bool reverse=false) {
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

void auto_intake_eject(int rotations=360, bool wait=true, int speed=200) {
  intake.resetPosition();
  intake.spinTo(-rotations, deg, speed, rpm, wait);
}

void auto_intake_grab(int rotations=360, bool wait=true, int speed=200) {
  intake.resetPosition();
  intake.spinTo(rotations, deg, speed, rpm, wait);
}


void autonomous(void) {
  vexcodeInit();

  LeftDriveSmart.setStopping(brakeType::hold);
  RightDriveSmart.setStopping(brakeType::hold);
  
  // new automation
  moveForward(-700, 150);
  wait(0.2, sec);
  turn_angle_2D(-36, 150);
  wait(0.2, sec);
  moveForward(-700, 200);
  wait(0.2, sec);
  turn_angle_2D(-62, 150);
  wait(0.2, sec);
  moveForward(-100, 100);
  wait(0.2, sec);
  auto_intake_eject(2500, false);
  wait(0.2, sec);
  moveForward(400); 
  wait(0.2, sec);


  moveForward(-300, 150);
  wait(0.2, sec);
  turn_angle_2D(180, 100);
  wait(0.2, sec);
  moveForward(-400, 150);
  wait(0.2, sec);
  turn_angle_2D(-35);
  wait(0.2, sec);
  auto_intake_grab(5000, false);
  moveForward(750, 125);
  wait(0.2, sec);
  moveForward(250, 40);
  wait(0.7, sec);
  moveForward(-100, 40);
  wait(0.2, sec);
  moveForward(100, 40);
  wait(0.7, sec);


  moveForward(-400, 150);
  wait(0.2, sec);
  turn_angle_2D(135, 150);
  wait(0.2, sec);
  moveForward(200, 150);
  wait(0.2, sec);
  auto_intake_eject(2500);



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

void usercontrol(void) {
  LeftDriveSmart.setStopping(brakeType::hold);
  RightDriveSmart.setStopping(brakeType::hold);
  Drivetrain.setDriveVelocity(MAX_DRIVETRAIN_VELOCITY_RPM, rpm);


  // User control code here, inside the loop
  while (1) {
    RightDriveSmart.spin(vex::directionType::fwd,
                         get_speed_direction("right") * MAX_DRIVETRAIN_VELOCITY_RPM / 100,
                         vex::velocityUnits::rpm);
    LeftDriveSmart.spin(vex::directionType::fwd, get_speed_direction("left") * MAX_DRIVETRAIN_VELOCITY_RPM / 100,
                        vex::velocityUnits::rpm);


    // Controller2.ButtonX.pressed([]()
    //                             { openAndCloseShield(); });

    Controller2.ButtonA.pressed([]()
                                { wingsFunction(); });
    
    if (Controller2.ButtonR2.pressing()) {
      autonomous();
    }

    if (Controller2.ButtonR1.pressing()) {
      moveIntakeToInside();
    } else if (Controller2.ButtonL1.pressing()) {
      moveIntakeToOutside();
    } else {
      stopIntake();
    }

    if (Controller2.ButtonB.pressing()) {
      switch_control_direction(&controllerStartTimer);
    }

    wait(20, msec);
  }
}


int main() {
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

struct {
  int X = 0;
  int Y = 0;
  float theta = 0.0;
} position;

void track_location() {
  // the distance between the front center and the right and left wheel
  dtheta = 0;

  // get the distance travelled by the left and right wheel by getting the change in angle then to mm
  float dr = (rightFrontMotor.position(deg) - current_motor_angle_right) * (M_PI * wheeldiam) / 360;
  float dl = (leftFrontMotor.position(deg) - current_motor_angle_left) * (M_PI * wheeldiam) / 360;

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
