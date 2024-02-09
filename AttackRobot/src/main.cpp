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

struct {
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
void auto_face_greentriball(vision visionSensor, int error_margin = 50,
                            int camera_x = 158);




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
/*                             Autonomous Phase                              */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
void autonomous(void) {
  // Insert autonomous user code here.
  Drivetrain.setDriveVelocity(25, pct);
  Drivetrain.setTurnVelocity(25, pct);
  Drivetrain.driveFor(500, mm, true);
  Drivetrain.turnFor(50, rotationUnits::deg);

  // drivetrain code to be tested
}




/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                            User Control Phase                             */
/*                                                                           */
/*                                                                           */
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
  double turn_importance = 0.5;



  /***************************************************************************/
  /*        Build functionalities for your Joystick inside the loop          */
  /***************************************************************************/

  while (1) {
    /*************************************************************************/
    /*                         Quick Buttons Set Up                          */
    /*************************************************************************/

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



    /*************************************************************************/
    /*                            Vision Sensor                              */
    /*************************************************************************/

    // visionSensor.takeSnapshot(GREENTRIBALL);
    // if (visionSensor.largestObject.exists) {
      // screen.printAt(10, 30, "Green Triball X: %d ",
      // visionSensor.largestObject.centerX); screen.printAt(230, 30, "Y: %d ",
      // visionSensor.largestObject.centerY); screen.printAt(310, 30, "W: %d ",
      // visionSensor.largestObject.width); screen.printAt(370, 30, "H %d ",
      // visionSensor.largestObject.height);
    // }



    /*************************************************************************/
    /*                              Drivetrain                               */
    /*************************************************************************/

    //todo Need to Fix Left A and Right B motors since Left B, Right A are fixed

    // double turn_value = Controller1.Axis1.position();
    // double turn_volts = turn_value * 0.12;

    // double forward_value = Controller1.Axis3.position();
    // double forward_volts = forward_value * 0.12 * (1 - (std::abs(turn_volts)/12.0) * turn_importance);

    // leftMotorA.spin(fwd, forward_volts - turn_volts, volt);
    // rightMotorA.spin(reverse, forward_volts - turn_volts, volt);
    
    // leftMotorB.spin(fwd, forward_volts + turn_volts, volt);
    // rightMotorB.spin(reverse, forward_volts + turn_volts, volt);

    // ----------------------------- code 2------------------------------------

    // if (Controller1.Axis1.position(pct) > 10) {
    //   LeftDriveSmart.spin(fwd, Controller1.Axis1.position(), pct);
    //   RightDriveSmart.spin(reverse, Controller1.Axis1.position(), pct);
    // }

    // if (Controller1.Axis1.position(pct) < -10) {
    //   LeftDriveSmart.spin(reverse, Controller1.Axis1.position(), pct);
    //   RightDriveSmart.spin(fwd, Controller1.Axis1.position(), pct);
    // }
    
    // if (Controller1.Axis3.position(pct) > 10) {
    //   LeftDriveSmart.spin(fwd, Controller1.Axis1.position(), pct);
    //   RightDriveSmart.spin(fwd, Controller1.Axis1.position(), pct);
    // }

    // if (Controller1.Axis3.position(pct) < -10) {
    //   LeftDriveSmart.spin(reverse, Controller1.Axis1.position(), pct);
    //   RightDriveSmart.spin(reverse, Controller1.Axis1.position(), pct);
    // }

    // ----------------------------- code 3------------------------------------

    obstacle_distance = 100000;
    if (frontDistance.isObjectDetected()) {
      obstacle_distance = frontDistance.objectDistance(mm);
    }

    if (obstacle_distance > 150 || Controller1.Axis2.position() < 0) {
      RightDriveSmart.spin(vex::directionType::fwd,
                           get_speed_direction("right"),
                           vex::velocityUnits::pct);
      LeftDriveSmart.spin(vex::directionType::fwd, get_speed_direction("left"),
                          vex::velocityUnits::pct);
    } else {
      RightDriveSmart.stop(vex::brakeType::brake);
      LeftDriveSmart.stop(vex::brakeType::brake);
    }



    /*************************************************************************/
    /*         Tracking the Motors Angle && the Heading of the Robot         */
    /*************************************************************************/
    current_motor_angle_left = leftMotorA.position(deg);
    current_motor_angle_right = rightMotorA.position(deg);
    position.theta = inertial_sensor.heading() * 180 / M_PI;



    stop = 1;


    /*************************************************************************/
    /*                            Temp Code                                  */
    /*************************************************************************/

    screen.printAt(10, 10, "Dist: %f", obstacle_distance);
    screen.printAt(10, 60, "Reverse Control: %d", reverserControl);
    screen.printAt(10, 90, "Inertial Sensor heading: %f",
                   inertial_sensor.heading(degrees));
    screen.printAt(10, 120, "GPS X:%lf Y:%lf", gps_sensor.xPosition(mm),
                   gps_sensor.yPosition(mm));
    screen.printAt(10, 150, "GPS Quality: %lf", gps_sensor.quality());


    /******************************* END ************************************/
    wait(20, msec); // Sleep the task to prevent wasted resources.
  }
}  // the end of the user control mode




/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                Set Up Competition Functions & Callbacks                   */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting using an infinite loop.
  while (true) {
    wait(100, msec);
  }
} // End of the Main function




/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                       Custom Function Definitions                         */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/


// function for automatically staring the robot toward a detected green triball
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



// function to get the speed for the rotation of the motors depending on wether
// the controls are reversed or not reversing the controls specifically the
// Axis2 is done by pressing R2 This can be useful for easier control of the
// robots when its rotated 180 degrees
float get_speed_direction(const char *side) {
  if (reverserControl) {
    if (strcmp(side, "right")) {
      return -(Controller1.Axis2.position() + Controller1.Axis4.position());
    }
    
    if (strcmp(side, "left")) {
      return -(Controller1.Axis2.position() - Controller1.Axis4.position());
    }
  }

  if (strcmp(side, "right")) {
    return Controller1.Axis2.position() - Controller1.Axis4.position();
  }
  
  if (strcmp(side, "left")) {
    return Controller1.Axis2.position() + Controller1.Axis4.position();
  }
  
  return 0;
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