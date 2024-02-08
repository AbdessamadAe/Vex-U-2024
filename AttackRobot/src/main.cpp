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

#include <cmath>
#include <ctime>

#include "objectDtection.h"
#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
motor leftMotorA = motor(PORT1, ratio18_1, false);
motor leftMotorB = motor(PORT11, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);

motor rightMotorA = motor(PORT12, ratio18_1, false);
motor rightMotorB = motor(PORT4, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);

drivetrain Drivetrain =
    drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);

controller Controller1 = controller(primary);
distance frontDistance = distance(PORT19);
brain::lcd screen = vex::brain::lcd();
vision visionSensor = vision(PORT10);
triport Threewireport = triport(PORT22);
limit switch_sensor = limit(Threewireport.A);
inertial inertial_sensor = inertial(PORT16);
gps gps_sensor = gps(PORT18);

// custom global variables
double dist = 100000;
bool reverserControl = false;
time_t controllerStartTimer = time(NULL);
int current_motor_angle_left = 0;
int current_motor_angle_right = 0;
float d = 0;
float rw = 140;
float wheeldiam = 101.6;
float dtheta = 0;
int stop = 0;

struct {
  int X = 0;
  int Y = 0;
  float theta = 0.0;
} position;

// functions prototypes
void face_angle_smooth(float target_angle = 50.0, float acceptable_error = 5);

// custom functions

// function to get the speed for the rotation of the motors depending on wether
// the controls are reversed or not reversing the controls specifically the
// Axis2 is done by pressing R2 This can be usefull for easier control of the
// robots when its rotated 180 degrees
float get_speed_direction(const char *side) {
  if (reverserControl) {
    if (strcmp(side, "right")) {
      return -(Controller1.Axis2.position() + Controller1.Axis4.position());
    } else if (strcmp(side, "left")) {
      return -(Controller1.Axis2.position() - Controller1.Axis4.position());
    }
  }
  if (strcmp(side, "right")) {
    return Controller1.Axis2.position() - Controller1.Axis4.position();
  } else if (strcmp(side, "left")) {
    return Controller1.Axis2.position() + Controller1.Axis4.position();
  } else {
    return 0;
  }
}

// function to switch the controller flag after a 1 second cooldown
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

// function for automatically stearing the robot toward a detected green triball
// given the visionSensor object that took the snapshot
// It will also move the robot toward the triball until collision is detected
// with the front switch sensor take into account the location of the camera and
// a margin error (optional)
void auto_face_greentriball(vision visionSensor, int error_margin = 50,
                            int camera_x = 158) {
  if (visionSensor.largestObject.exists) {
    int triball_x = visionSensor.largestObject.centerX;
    if (triball_x <= (camera_x - error_margin)) {
      // turn left
      float motor_speed = 25 * triball_x / camera_x;
      RightDriveSmart.spin(vex::directionType::fwd, motor_speed, pct);
      LeftDriveSmart.spin(vex::directionType::rev, motor_speed, pct);
    } else if (triball_x >= (camera_x + error_margin)) {
      // turn right
      float motor_speed = 25 * (1 - camera_x / triball_x);
      RightDriveSmart.spin(vex::directionType::rev, motor_speed, pct);
      LeftDriveSmart.spin(vex::directionType::fwd, motor_speed, pct);
    } else if (!switch_sensor.pressing()) {
      RightDriveSmart.spin(fwd, 25, pct);
      LeftDriveSmart.spin(fwd, 25, pct);
    } else {
      // break
      RightDriveSmart.stop(vex::brakeType::brake);
      LeftDriveSmart.stop(vex::brakeType::brake);
    }
  }
  return;
}

void face_angle_smooth(float target_angle, float acceptable_error) {
  float current_angle = inertial_sensor.heading();
  float error_angle = current_angle - target_angle;
  float motor_speed;
  while (std::abs(error_angle) > acceptable_error) {
    if (error_angle > 0) {
      if (error_angle <= 180) {
        // turn left
        motor_speed = error_angle * 0.5;
      } else {
        // turn right
        motor_speed = -(360 - error_angle) * 0.5;
      }
    } else {
      if (-error_angle <= 180) {
        // turn right
        motor_speed = error_angle * 0.5;
      } else {
        motor_speed = (360 + error_angle) * 0.5;
      }
    }

    RightDriveSmart.spin(fwd, motor_speed, pct);
    LeftDriveSmart.spin(fwd, -motor_speed, pct);

    current_angle = inertial_sensor.heading();
    error_angle = current_angle - target_angle;
    // screen.printAt(10, 180, "Turning smoothly ....");

    // break the automated turning in case the robot stuck
    if (Controller1.ButtonL1.pressing()) {
      return;
    }
    wait(20, msec);
  }
}

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = false;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  return;
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

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  Drivetrain.setDriveVelocity(25, pct);
  Drivetrain.setTurnVelocity(25, pct);
  Drivetrain.driveFor(500, mm, true);
  Drivetrain.turnFor(50, rotationUnits::deg);

  // drivetrain code to be tested

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

void usercontrol(void) {
  LeftDriveSmart.setStopping(brakeType::hold);
  RightDriveSmart.setStopping(brakeType::hold);
  Drivetrain.setDriveVelocity(50, pct);
  Drivetrain.setTurnVelocity(25, pct);
  inertial_sensor.calibrate();
  gps_sensor.calibrate();

  while (inertial_sensor.isCalibrating()) {
    wait(50, msec);
  }
  while (gps_sensor.isCalibrating()) {
    wait(50, msec);
  }

  rightMotorA.setPosition(0, deg);
  leftMotorA.setPosition(0, deg);

  /******************************** Joystick *********************************/
  /*                                                                         */
  /*                User control code here, inside the loop                  */
  /*                                                                         */
  /***************************************************************************/
  while (1) {
    if (Controller1.ButtonR1.pressing()) {
      autonomous();
      }

    if (Controller1.ButtonR2.pressing()) {
      switch_control_direction(&controllerStartTimer);
    }

    if (Controller1.ButtonL1.pressing()) {
      face_angle_smooth();
    }

    if (Controller1.ButtonL2.pressing()) {
      auto_face_greentriball(visionSensor);
    }

    visionSensor.takeSnapshot(GREENTRIBALL);
    if (visionSensor.largestObject.exists) {
      // screen.printAt(10, 30, "Green Triball X: %d ",
      // visionSensor.largestObject.centerX); screen.printAt(230, 30, "Y: %d ",
      // visionSensor.largestObject.centerY); screen.printAt(310, 30, "W: %d ",
      // visionSensor.largestObject.width); screen.printAt(370, 30, "H %d ",
      // visionSensor.largestObject.height);
    }

    screen.printAt(10, 60, "Reverse Control: %d", reverserControl);
    screen.printAt(10, 90, "Inertial Sensor heading: %f",
                   inertial_sensor.heading(degrees));
    screen.printAt(10, 120, "GPS X:%lf Y:%lf", gps_sensor.xPosition(mm),
                   gps_sensor.yPosition(mm));
    screen.printAt(10, 150, "GPS Quality: %lf", gps_sensor.quality());


    dist = 100000;
    if (frontDistance.isObjectDetected()) {
      dist = frontDistance.objectDistance(mm);
    }

    screen.printAt(10, 10, "Dist: %f", dist);

    if (dist > 150) {
      RightDriveSmart.spin(vex::directionType::fwd,
                           get_speed_direction("right"),
                           vex::velocityUnits::pct);
      LeftDriveSmart.spin(vex::directionType::fwd, get_speed_direction("left"),
                          vex::velocityUnits::pct);
    }

    else if (Controller1.Axis2.position() < 0) {
      RightDriveSmart.spin(vex::directionType::fwd,
                           get_speed_direction("right"),
                           vex::velocityUnits::pct);
      LeftDriveSmart.spin(vex::directionType::fwd, get_speed_direction("left"),
                          vex::velocityUnits::pct);
    } else {
      RightDriveSmart.stop(vex::brakeType::brake);
      LeftDriveSmart.stop(vex::brakeType::brake);
    }

    current_motor_angle_left = leftMotorA.position(deg);
    current_motor_angle_right = rightMotorA.position(deg);
    position.theta = inertial_sensor.heading() * 180 / M_PI;

    stop = 1;

    // Sleep the task for a short amount of time to prevent wasted resources.
    wait(20, msec);
  }
}  // the end of the usercontrol mode

// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}