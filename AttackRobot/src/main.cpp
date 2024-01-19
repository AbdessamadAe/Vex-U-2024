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

  #include "vex.h"
  #include "objectDtection.h"
  #include <ctime>
  #include <cmath>

  using namespace vex;

  // A global instance of competition
  competition Competition;

  // define your global instances of motors and other devices here
  motor leftMotorA = motor(PORT1, ratio18_1, false);
  motor leftMotorB = motor(PORT11, ratio18_1, false);
  motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
  motor rightMotorA = motor(PORT12, ratio18_1, true);
  motor rightMotorB = motor(PORT4, ratio18_1, true);
  motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
  drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
  controller Controller1 = controller(primary);
  distance frontDistance = distance(PORT19);
  brain::lcd screen = vex::brain::lcd();
  vision visionSensor = vision(PORT10);
  triport Threewireport = triport(PORT22);
  limit switch_sensor = limit(Threewireport.A);
  inertial inertial_sensor = inertial(PORT16);

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

  struct
  {
    int X = 0;
    int Y = 0;
    float theta = 0.0;
  } position;

  // functions prototypes
  void face_angle_smooth(float target_angle = 50.0, float acceptable_error = 5);

  // custom functions

  // function to get the speed for the rotation of the motors depending on wether the controls are reversed or not
  // reversing the controls specifically the Axis2 is done by pressing R2
  // This can be usefull for easier control of the robots when its rotated 180 degrees
  float get_speed_direction(const char *side)
  {
    if (reverserControl)
    {
      if (strcmp(side, "right"))
      {
        return -(Controller1.Axis2.position() - Controller1.Axis4.position());
      }
      else if (strcmp(side, "left"))
      {
        return -(Controller1.Axis2.position() + Controller1.Axis4.position());
      }
    }
    if (strcmp(side, "right"))
    {
      return Controller1.Axis2.position() + Controller1.Axis4.position();
    }
    else if (strcmp(side, "left"))
    {
      return Controller1.Axis2.position() - Controller1.Axis4.position();
    }
    else
    {
      return 0;
    }
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

  // function for automatically stearing the robot toward a detected green triball
  // given the visionSensor object that took the snapshot
  // It will also move the robot toward the triball until collision is detected with the front switch sensor
  // take into account the location of the camera and a margin error (optional)
  void auto_face_greentriball(vision visionSensor, int error_margin = 50, int camera_x = 158)
  {
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
      screen.printAt(10, 180, "Turning smoothly ....");

      // break the automated turning in case the robot stuck
      if (Controller1.ButtonL1.pressing())
      {
        return;
      }
      wait(20, msec);
    }
  }

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

    screen.printAt(10, 120, "X: %d", position.X);
    screen.printAt(10, 150, "Y: %d", position.Y);
    screen.printAt(10, 180, "Theta: %f", position.theta*180/M_PI);
    screen.printAt(10, 210, "dtheta: %f", dtheta*180/M_PI);
  }

  void turnRobotToFace(float angle) {
    Drivetrain.turnFor(angle* 180 / M_PI, degrees);
}

void moveRobotForward(float distance) {
    Drivetrain.driveFor(distance, mm);
}


void moveToCoordinate(float targetX, float targetY) {
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
    while (angleToTurn > M_PI) angleToTurn -= 2 * M_PI;
    while (angleToTurn < -M_PI) angleToTurn += 2 * M_PI;

    if (stop == 1) {
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

  void pre_auton(void)
  {
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

  void autonomous(void)
  {
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

    rightMotorA.setPosition(0, deg);
    leftMotorA.setPosition(0, deg);


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

      /*---------------------------------------------------------------------------*/
      /*                              Drivetrain Control                            */
      /*---------------------------------------------------------------------------*/
      // Axis4 = Left/Right on the left joystick
      // Axis2 = Up/Down on the right joystick
      // controller1.Axis4.position() is for forward/backward movement
      // The controller1.Axis4.position() is preceeded by a minus sign to reverse
      // the direction of the right motors. This is so that the robot turns for e.g right when
      // the joystick is pushed to right by rotating the righ wheels backwords and the
      // left wheels forwards.
      dist = 100000;
      if (frontDistance.isObjectDetected())
      {
        dist = frontDistance.objectDistance(mm);
      }

      screen.printAt(10, 10, "Dist: %f", dist);

      if (dist > 150)
      {
        RightDriveSmart.spin(vex::directionType::fwd,
                            get_speed_direction("right"),
                            vex::velocityUnits::pct);
        LeftDriveSmart.spin(vex::directionType::fwd,
                            get_speed_direction("left"),
                            vex::velocityUnits::pct);
      }

      else if (Controller1.Axis2.position() < 0)
      {
        RightDriveSmart.spin(vex::directionType::fwd,
                            get_speed_direction("right"),
                            vex::velocityUnits::pct);
        LeftDriveSmart.spin(vex::directionType::fwd,
                            get_speed_direction("left"),
                            vex::velocityUnits::pct);
      }
      else
      {
        RightDriveSmart.stop(vex::brakeType::brake);
        LeftDriveSmart.stop(vex::brakeType::brake);
      }

      track_location();

      current_motor_angle_left = leftMotorA.position(deg);
      current_motor_angle_right = rightMotorA.position(deg);
      position.theta = inertial_sensor.heading() * 180 / M_PI;

      moveToCoordinate(1037, 393);
      moveToCoordinate(10, 393);
      stop = 1;


      visionSensor.takeSnapshot(GREENTRIBALL);
      if (visionSensor.largestObject.exists)
      {
        screen.printAt(10, 30, "Green Triball X: %d ", visionSensor.largestObject.centerX);
        screen.printAt(230, 30, "Y: %d ", visionSensor.largestObject.centerY);
        screen.printAt(310, 30, "W: %d ", visionSensor.largestObject.width);
        screen.printAt(370, 30, "H %d ", visionSensor.largestObject.height);
      }

      if (Controller1.ButtonR2.pressing())
      {
        switch_control_direction(&controllerStartTimer);
      }
      screen.printAt(10, 60, "Reverse Control: %d", reverserControl);

      if (Controller1.ButtonL2.pressing())
      {
        auto_face_greentriball(visionSensor);
      }
      if (Controller1.ButtonL1.pressing())
      {
        face_angle_smooth();
      }
      if (Controller1.ButtonR1.pressing())
      {
        autonomous();
      }

      screen.printAt(10, 90, "Inertial Sensor heading: %f", inertial_sensor.heading(degrees));

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
