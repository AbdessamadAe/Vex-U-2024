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
#include <cmath>

#include "vex.h"


using namespace vex;

brain Brain;

digital_out DigitalOutA = digital_out(Brain.ThreeWirePort.B);
digital_out DigitalOutB = digital_out(Brain.ThreeWirePort.A);

namespace wheels_circumferences_mm {
const double k200Mm = 200;
const double k2_75Inch = 219.44;
const double k3_25Inch = 299.24;
const double k4Inch = 319.19;
const double k5Inch = 398.98;
const double k6Inch = 478.78;
}  // namespace wheels_circumferences_mm

/*----------------------------------------------------------------------------*/
/*                            Robot Specs (change them)                       */
/*----------------------------------------------------------------------------*/
namespace robot_specs {
// Mechanical Advantage
const int kDrivenGear = 24;
const int kDrivingGear = 84;
const double kGearRatio = (double)kDrivenGear / kDrivingGear;
const int kTargetedVelocityInRPM = 357;
const int kMaxDrivetrainVelocityInRPM = kTargetedVelocityInRPM * kGearRatio;
const double kMaxDrivetrainVelocityInPCT =
    (double)((kMaxDrivetrainVelocityInRPM * 100) / (double)200) / (double)100;

const int kMaxIntakeVelocityInRPM = 170;  // to use only 85% of motor's capacity

// Wheels and Robot Dimensions
const double kWheelCircumferenceInMM = wheels_circumferences_mm::k4Inch;

// Track width is the distance between the robot’s right wheels’ center point
// and the robot’s left wheels’ center point.
const double kWheelTrackWidthInMM = 298;

// Wheelbase is the distance between the shafts of the two drive wheels (fear
// front and far back) on the robot’s side.
const double kWheelbaseInMM = 255;

// Sensors
const double kGPSXOffsetInMM = 0;  // offset from the center of the robot
const double kGPSYOffsetInMM = 0;  // offset from the center of the robot

// 0 deg if the GPS camera is set in forward direction of the robot
// 90 deg if the GPS camera is set in the right direction of the robot
const double kGPSAngleOffsetInDegree = 0;  // preferable to be at 180 degree
}  // namespace robot_specs

/*----------------------------------------------------------------------------*/
/*                                Global Instances                            */
/*----------------------------------------------------------------------------*/
competition Competition;  // A global instance of competition
controller Controller2 = controller(primary);

// Drivetrain
motor leftFrontMotor = motor(PORT1, ratio18_1, false);
motor leftBackMotor = motor(PORT3, ratio18_1, false);
motor rightFrontMotor = motor(PORT19, ratio18_1, true);
motor rightBackMotor = motor(PORT4, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(leftFrontMotor, leftBackMotor);
motor_group RightDriveSmart = motor_group(rightFrontMotor, rightBackMotor);
drivetrain Drivetrain = drivetrain(
    LeftDriveSmart, RightDriveSmart, robot_specs::kWheelCircumferenceInMM,
    robot_specs::kWheelTrackWidthInMM, robot_specs::kWheelbaseInMM, mm,
    robot_specs::kGearRatio);

gps DrivetrainGPS =
    gps(PORT20, robot_specs::kGPSXOffsetInMM, robot_specs::kGPSYOffsetInMM, mm,
        robot_specs::kGPSAngleOffsetInDegree);

// Intake
motor intakeMotor = motor(PORT10, ratio18_1, false);



// Wing
// motor rightWingMotor = motor(PORT15, ratio18_1, false);
// motor leftWingMotor = motor(PORT16, ratio18_1, true);
// motor_group wingMotors = motor_group(rightWingMotor, leftWingMotor);

/*----------------------------------------------------------------------------*/
/*                                Global Constants                            */
/*----------------------------------------------------------------------------*/
const float TURN_ANGLE_MOTOR_RATIO = 1.05;
const float MOVE_MOTOR_RATIO = 0.31;

/*----------------------------------------------------------------------------*/
/*                                Global Variables                            */
/*----------------------------------------------------------------------------*/
bool reverserControl = false;
time_t controllerStartTimer = time(NULL);
// int WingsExtended = 0;

/*----------------------------------------------------------------------------*/
/*                                Intake Functions                            */
/*----------------------------------------------------------------------------*/
void moveIntakeToInside() {
  intakeMotor.spin(vex::directionType::rev,
                   robot_specs::kMaxIntakeVelocityInRPM,
                   vex::velocityUnits::rpm);
}

void moveIntakeToOutside() {
  intakeMotor.spin(vex::directionType::fwd,
                   robot_specs::kMaxIntakeVelocityInRPM,
                   vex::velocityUnits::rpm);
}

void stopIntake() { intakeMotor.stop(); }


/*---------------------------------------------------------------------------*/
/*                                Wings Pneu Functions                            */
/*---------------------------------------------------------------------------*/
bool on = false;

void pnematicWings(bool v) {
  DigitalOutA.set(v);
  DigitalOutB.set(v);

}

/*---------------------------------------------------------------------------*/
/*                                Wings Functions                            */
/*---------------------------------------------------------------------------*/
// void wingsFunction() {
//   if (WingsExtended == 0) {
//     wingMotors.setVelocity(20, pct);
//     wingMotors.spinFor(directionType::fwd, 145, rotationUnits::deg);
//     WingsExtended = 1;
//   }

//   else if (WingsExtended == 1) {
//     wingMotors.setVelocity(20, pct);
//     wingMotors.spinFor(directionType::fwd, -145, rotationUnits::deg);
//     WingsExtended = 0;
//     rightWingMotor.setBrake(brakeType::hold);
//     leftWingMotor.setBrake(brakeType::hold);
//   }
// }

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

  LeftDriveSmart.setStopping(brakeType::coast);
  RightDriveSmart.setStopping(brakeType::coast);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                            Autonomous Functions                           */
/*---------------------------------------------------------------------------*/
double mm_to_deg(int distance_mm) {
  double rev = distance_mm / robot_specs::kWheelCircumferenceInMM;
  return rev * 360;
}

void moveForward(int distance_mm,
                 int speed = robot_specs::kMaxDrivetrainVelocityInRPM) {
  RightDriveSmart.resetPosition();
  LeftDriveSmart.resetPosition();

  RightDriveSmart.setTimeout(3, sec);
  LeftDriveSmart.setTimeout(3, sec);

  double dist_deg = -mm_to_deg(distance_mm) * MOVE_MOTOR_RATIO;  // need to be fine tuned

  RightDriveSmart.spinTo(dist_deg, deg, speed, rpm, false);
  LeftDriveSmart.spinTo(dist_deg, deg, speed, rpm, true);
}

void turn_angle_2D(int angle,
                   int speed = robot_specs::kMaxDrivetrainVelocityInRPM) {
  RightDriveSmart.resetPosition();
  LeftDriveSmart.resetPosition();

  double deg_angle = -angle * TURN_ANGLE_MOTOR_RATIO;
  LeftDriveSmart.spinTo(deg_angle, deg, speed, rpm, false);
  RightDriveSmart.spinTo(-deg_angle, deg, speed, rpm);
}

void turn_angle_1D(int angle,
                   int speed = robot_specs::kMaxDrivetrainVelocityInRPM,
                   bool reverse = false) {
  RightDriveSmart.resetPosition();
  LeftDriveSmart.resetPosition();

  double deg_angle = -angle * TURN_ANGLE_MOTOR_RATIO;

  if (reverse) {
    if (angle > 0) {
      LeftDriveSmart.spinTo(-deg_angle * 2, deg, speed, rpm);
    } else {
      RightDriveSmart.spinTo(-deg_angle * 2, deg, speed, rpm);
    }
  } else {
    if (angle > 0) {
      LeftDriveSmart.spinTo(deg_angle * 2, deg, speed, rpm);
    } else {
      RightDriveSmart.spinTo(deg_angle * 2, deg, speed, rpm);
    }
  }
}

void auto_intake_eject(int rotations = 360, bool wait = true,
                       int speed = robot_specs::kMaxIntakeVelocityInRPM) {
  intakeMotor.resetPosition();
  intakeMotor.spinTo(rotations, deg, speed, rpm, wait);
}

void auto_intake_grab(int rotations = 360, bool wait = true,
                      int speed = robot_specs::kMaxIntakeVelocityInRPM) {
  intakeMotor.resetPosition();
  intakeMotor.spinTo(-rotations, deg, speed, rpm, wait);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void auton_part1(){

  //starting from the limit allowed under the horizental bar, the green ball will directly in front and touching it 
  // auto_intake_grab(5000, false, 170);
  moveForward(-100, 70);
  wait(0.1, sec);
  moveForward(-1200, 150);
  wait(0.2, sec);
  turn_angle_2D(-117, 70);
  wait(0.2, sec);
  //open wings
  moveForward(-1370, 150);
  wait(0.2, sec);
  //close wings
  return;
}

void auton_part2(){
  //starting ideally near the 2 strip white lines at the limit with the black bar 
  turn_angle_2D(-82, 70);
  wait(0.2, sec);
  moveForward(200, 150);
  wait(0.2, sec);
  auto_intake_eject(600, true, 200);
  wait(0.2, sec);
  moveForward(-400, 150);
  wait(0.2, sec);
  auto_intake_eject(600, true, 200);
  wait(0.2, sec);
  moveForward(500, 200);
  wait(0.2, sec);

  moveForward(-200, 150);
  wait(0.2, sec);
  turn_angle_2D(157, 70);
  wait(0.2, sec);
  return;
  //near the goal facing the next green triball 
  
}

void auton_part3(){
  moveForward(750, 150);
  wait(0.2, sec);
  //grab ball in
  auto_intake_grab(1200, false, 200);
  wait(0.2, sec);
  moveForward(50, 70);
  wait(1, sec);
  turn_angle_2D(-165, 70);
  wait(0.2, sec);
  auto_intake_eject(1500, false, 200);
  moveForward(680, 150);
  wait(0.2, sec);
  auto_intake_eject(1500, true, 200);
  wait(0.2, sec);
  moveForward(-200, 150);
  wait(0.2, sec);
  turn_angle_2D(-10, 70);
  wait(0.2, sec);
  moveForward(400, 200);
  
}


void autonomous(void) {
  vexcodeInit();

  // LeftDriveSmart.setStopping(brakeType::brea);
  // RightDriveSmart.setStopping(brakeType::hold);

  //auton part 1
  auton_part1();
  auton_part2();
  auton_part3();
  
}

/*----------------------------------------------------------------------------*/
/*                    Drive Control Specific Functions                        */
/*----------------------------------------------------------------------------*/
double get_speed_direction(const char *side) {
  double horiz_nerf = 0.8;
  if (reverserControl) {
    if (strcmp(side, "right")) {
      return Controller2.Axis3.position(pct) - Controller2.Axis1.position(pct)*horiz_nerf;
    }

    if (strcmp(side, "left")) {
      return Controller2.Axis3.position(pct) + Controller2.Axis1.position(pct)*horiz_nerf;
    }
  }

  if (strcmp(side, "right")) {
    return -Controller2.Axis3.position(pct) - Controller2.Axis1.position(pct)*horiz_nerf;
  }

  if (strcmp(side, "left")) {
    return -Controller2.Axis3.position(pct) + Controller2.Axis1.position(pct)*horiz_nerf;
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
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
  LeftDriveSmart.setStopping(brakeType::hold);
  RightDriveSmart.setStopping(brakeType::hold);
  Drivetrain.setDriveVelocity(robot_specs::kMaxDrivetrainVelocityInRPM,
                              vex::velocityUnits::rpm);

  DigitalOutA.set(false);
  DigitalOutB.set(false);
  

  // User control code here, inside the loop
  while (1) {
    // we multiple by a targeted velocity and divide by 100, so we can convert
    // the percentage into rpm. All this to not exceed the targeted velocity.
    RightDriveSmart.spin(
        vex::directionType::fwd,
        get_speed_direction("right") * robot_specs::kMaxDrivetrainVelocityInPCT,
        vex::velocityUnits::pct);

    LeftDriveSmart.spin(
        vex::directionType::fwd,
        get_speed_direction("left") * robot_specs::kMaxDrivetrainVelocityInPCT,
        vex::velocityUnits::pct);

    // Controller2.ButtonA.pressed([]() { wingsFunction(); });

    if (Controller2.ButtonUp.pressing()) {
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

    Controller2.ButtonA.pressed([]() { pnematicWings(true); });
    Controller2.ButtonY.pressed([]() { pnematicWings(false); });

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
  while (true) {
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

  double magnitude = sqrt(x_distance_to_drive * x_distance_to_drive +
y_distance_to_drive * y_distance_to_drive);

  double angle = std::asin((std::sin(M_PI / 2.0) * x_distance_to_drive) /
magnitude) * 180.0 / M_PI;

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

  while (gps_sensor.xPosition(mm) != x_target || gps_sensor.yPosition(mm) !=
y_target) { if (gps_sensor.xPosition(mm) > x_target-0.05 ||
gps_sensor.xPosition(mm) < x_target+0.05) { return;
    }

    if (gps_sensor.yPosition(mm) > y_target-0.05 || gps_sensor.yPosition(mm) <
y_target+0.05) { return;
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

// unused functions
// ==============================================================================
int current_motor_angle_left = 0;
int current_motor_angle_right = 0;
double d = 0;
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

  // get the distance travelled by the left and right wheel by getting the
  // change in angle then to mm
  float dr = (rightFrontMotor.position(deg) - current_motor_angle_right) *
             (M_PI * wheeldiam) / 360;
  float dl = (leftFrontMotor.position(deg) - current_motor_angle_left) *
             (M_PI * wheeldiam) / 360;

  // calculating the change in the orientation of the robot
  dtheta = (dr - dl) / (2 * rw);
  // calulating the distance travelled by the entire robot

  if (std::abs(dtheta) <= 0.01) {
    d = (dr + dl) / 2;
  } else {
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