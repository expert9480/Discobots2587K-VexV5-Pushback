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

using namespace vex;

double x, y;
double inchtodegree = 2950 / 50;
double armRotation = 0.0;
bool pneumaticToggle = false;
double distInertFromBack = 3;

// A global instance of competition
competition Competition;

controller Controller = controller(primary);
motor BTR = motor(PORT3, ratio6_1, true);
motor BBR = motor(PORT2, ratio6_1, false);
motor FR = motor(PORT1, ratio6_1, false);
motor BTL = motor(PORT13, ratio6_1, false);
motor BBL = motor(PORT12, ratio6_1, true);
motor FL = motor(PORT11, ratio6_1, true);

motor_group Right = motor_group(BTL, BBL, FL);
motor_group Left = motor_group(BTR, BBR, FR);

inertial Inertial = inertial(PORT20);

smartdrive Drivetrain = smartdrive(Left, Right, Inertial, 259.3385, 386.08, 271.78, mm, 1);
float gear_ratio = 36 / 60;

motor Arm = motor(PORT10, ratio36_1, false);

motor Intake = motor(PORT8, ratio6_1, true);

motor_group AllM = motor_group(BTL, BBL, FL, BTR, BBR, FR, Arm, Intake);

rotation Rotation = rotation(PORT19, true);
// Defined every connection on the current robot
void brakeall () { //breaks all the base motors
  BTR.stop();
  BBR.stop();
  FR.stop();
  FL.stop();
  BBL.stop();
  BTL.stop();
  BTL.setBrake(hold);
  BTR.setBrake(hold);
  FR.setBrake(hold);
  FL.setBrake(hold);
  BBR.setBrake(hold);
  BBL.setBrake(hold);
}

void setallright (double speed) { //sets velocity of all the right-side motors on the robot base
  BTL.setVelocity(speed, percent);
  BBL.setVelocity(speed, percent);
  FL.setVelocity(speed, percent);
}

void setallleft (double speed) { //sets the velocity of all the left-side motors on the robot base
  FR.setVelocity(speed, percent);
  BTR.setVelocity(speed, percent);
  BBR.setVelocity(speed, percent);
}

void spinallright (enum vex::directionType dir) { //spins all the right-side motors on the robot base
  BTL.spin(dir);
  BBL.spin(dir);
  FL.spin(dir);
}

void spinallleft (enum vex::directionType dir) { //spins all the left-side motors on the robot base
  BBR.spin(dir);
  BTR.spin(dir);
  FR.spin(dir);
}

void spinallleft (enum vex::directionType dir, double amnt) { //spins all the left-side motors on the robot base
  FR.spinFor(forward, amnt, degrees);
  BTR.spinFor(forward, amnt, degrees);
  BBR.spinFor(forward, amnt, degrees);
}

void spinallright (enum vex::directionType dir, double amnt) { //spins all the left-side motors on the robot base
  FL.spinFor(forward, amnt, degrees);
  BBL.spinFor(forward, amnt, degrees);
  BTL.spinFor(forward, amnt, degrees);
}

void spinall (enum vex::directionType dir) { //spins all the motors on the robot base
  spinallleft(dir);
  spinallright(dir);
}

// define your global instances of motors and other devices here
void movefront (bool dir, double goal) {
  double kp = .05; //tune proportional constant
  double ki = .00001; //tune integral constant
  double error = 0; //distance from goal
  double result = 0; // final motor speed
  double totalerror = 0;
  double previousresult = 0;
  double accelerator = 2; //tune max change
  int index = 0;
  double initialerror = Left.position(degrees) - goal*inchtodegree;

  Left.resetPosition();
  if (dir == true) {
    while (index <= 150) {
        error = Left.position(degrees) - goal*inchtodegree;
        Brain.Screen.setCursor(15, 1);
        Brain.Screen.print("Error: %f", error);
        totalerror += error;
        if (fabs(error) <= .2*inchtodegree || index > 150) {
          brakeall();
          index = 0;
          break;
        }
        result = (error * kp) + (totalerror * ki);
        if (result - previousresult > accelerator) {
          result = previousresult + accelerator;
        }
        if (previousresult - result > accelerator) {
          result = previousresult - accelerator;
        }
        setallleft(-result);
        setallright(-result);
        spinall(forward);
        previousresult = result;
        wait (20, msec);
        index += 1;
    }
  } else if (dir == false) {
    while (index <= 150) {
        error = Left.position(degrees) + goal*inchtodegree;
        totalerror += error;
        if (fabs(error) <= .2*inchtodegree || index > 150) {
          brakeall();
          index = 0;
          break;
        }
        result = (error * kp) + (totalerror * ki);
        if (result - previousresult > accelerator) {
          result = previousresult + accelerator;
        }
        if (previousresult - result > accelerator) {
          result = previousresult - accelerator;
        }
        setallleft(-result);
        setallright(-result);
        spinall(forward);
        previousresult = result;
        wait (20, msec);
        index += 1;
    }
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

void pre_auton(void) 
{
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  Arm.setBrake(hold);
  vexcodeInit();
  Rotation.resetPosition();
  Rotation.setPosition(0.0, turns);
  Inertial.startCalibration();
  Inertial.calibrate();
  Inertial.resetHeading();
  Drivetrain.setTurnVelocity(600, rpm);
  Arm.setVelocity(100, rpm);
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

void togglePneumatic() {
  bool toggle = false;
  if (!pneumaticToggle) {
    toggle = false;
    pneumaticToggle = !pneumaticToggle;
    Pneumatic.set(false);
  } else {
    toggle = true;
    pneumaticToggle = !pneumaticToggle;
    Pneumatic.set(true);
  }
}

void intake()
{
  Intake.spin(forward, 600, rpm);
}

void outtake()
{
  Intake.spin(reverse, 600, rpm);
}

void stopAll()
{
  BTL.stop();
  BBL.stop();
  FL.stop();
  BTR.stop();
  BBR.stop();
  FR.stop();
}

void brakeAll()
{
  stopAll();
  BTR.setBrake(hold);
  BBR.setBrake(hold);
  FR.setBrake(hold);
  BTL.setBrake(hold);
  BBL.setBrake(hold);
  FL.setBrake(hold);
}

void armUp()
{
  Arm.spin(forward, 100, rpm);
}
void armDown()
{
  // Arm.spin(reverse, 100, rpm);
  Arm.spin(reverse, 100, rpm);
}

double getrotation(){
  return (lround(fmod(Inertial.rotation(),360)) + 540) % 360 - 180;
}


void armStop()
{
  Arm.stop();
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
// Drivetrain.drive(forward, 600, rpm);
// wait(0.2, sec);
// Drivetrain.drive(reverse, 600, rpm);
// wait(1.5, sec);

//length without ramp: 16
//width 16
//length with ramp: 21.5

//grabs three on middle corner left
movefront(true, 20);
Drivetrain.turnFor(20, degrees, false);
intake();
movefront(true, 19);
Intake.stop();

// movefront(false,6);
Drivetrain.turnFor(45, degrees, false);
movefront(true,8);
//arm up center top (17 inch bot)
movefront(true,5);
outtake();
wait(2,sec);
Intake.stop();
movefront(false,5);
//arm down

//go to left loader
// movefront(false,37);
// Drivetrain.turnFor(45+90,degrees,true);


stopAll();
}

//remmeber to set up PID on monday

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

  Arm.setBrake(hold);
  // Brain.Screen.print("Running driver control!");
  //  User control code here, inside the loop
  Left.setStopping(coast);
  Right.setStopping(coast);
  Pneumatic.set(false);
  while (1)
  {
    x = -1 * (pow(1.0471285480509, -1 * Controller.Axis1.position())) + pow(1.0471285480509, Controller.Axis1.position()); // exponential function for turning tuning
    y = -1 * (pow(1.0471285480509, -1 * Controller.Axis3.position())) + pow(1.0471285480509, Controller.Axis3.position());
    int threshold = 2;
    if (x <= threshold && x >= -threshold)
    {
      x = 0;
    }

    if (y <= threshold && y >= -threshold)
    {
      y = 0;
    }
    Left.setVelocity(((y - x) * 6), rpm); // multiply by 6 for RPM value and set wheel speeds
    Left.spin(forward);
    Right.setVelocity((y + x) * 6, rpm);
    Right.spin(forward);
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

void buttonPressedDown() {
  // Pneumatic.set(!Pneumatic);
  // Pneumatic = !Pneumatic;
  togglePneumatic();
}

void buttonPressedL2()
{
  Pneumatic.set(true);
  while (Controller.ButtonL2.pressing())
  {
    armUp();
    wait(20.0, msec);
  }
  armStop();
}

void buttonPressedR2()
{
  while (Controller.ButtonR2.pressing())
  {
    armDown();
    wait(20.0, msec);
  }
  armStop();
}

void buttonPressedL1()
{
  intake();
}

void buttonPressedR1()
{
  outtake();
}


void buttonPressedY()
{
  Intake.stop();
}

void buttonPressedX() {
  Intake.stop();
  /*
  const double target = 0.7;
  const double tol = 0.002; // tolerance to avoid oscillation
  int safety = 0;           // safety counter to avoid infinite loop

  double current = Rotation.angle(turns);
  if (current >= target - tol && current <= target + tol) {
    armStop();
    return;
  }

  if (current < target) {
    armUp();
    while (Rotation.angle(turns) < target - tol && (++safety) < 2000) {
      wait(5, msec);
    }
  } else {
    armDown();
    while (Rotation.angle(turns) > target + tol && (++safety) < 2000) {
      wait(5, msec);
    }
  }

  armStop();
*/
  

  // armUp();
  // armUp(0.4);
  

  // Controller.Screen.clearScreen();
  // while (Rotation.angle(degrees) < targetangleH) {
  //   double currentAngle = Rotation.angle(degrees);

  //   Controller.Screen.clearLine();
  //   Controller.Screen.print(currentAngle);

  //   if (currentAngle < targetangleH) {
  //     armUp();
  //   } else if (currentAngle == targetangleH)
  //   // } else
  //   // } else if ((currentAngle >= 356.0) || (currentAngle <= 357.0))
  //   {
  //     armStop();
  //   }

  //   wait(10, msec);  // Prevent CPU overload
  // }

  // armStop();  // Final stop to ensure motor halts
}

void buttonPressedA() {
  /*const double target = 0.5;
  const double tol = 0.0005; // very small tolerance; increase if jitter occurs

  double current = Rotation.angle(turns);
  // already at target?
  if (current >= target - tol && current <= target + tol) {
    armStop();
    return;
  }

  // choose direction and run until we cross the target
  if (current < target) {
    armUp();
    while (Rotation.angle(turns) < target - tol) {
      wait(5, msec);
    }
  } else { // current > target
    armDown();
    while (Rotation.angle(turns) > target + tol) {
      wait(5, msec);
    }
  }

  armStop();
  */
  Intake.spinFor(forward, .35, turns, false);
}

void buttonPressedB() {

  Intake.stop();
/*
  if (Rotation.angle(turns) > 0.0) {
    armDown();
    while (Rotation.angle(turns) > 0.0) {
      wait(5, msec);
    }
  }
   armStop();
*/
/*
const double target = 0.0;
  const double tol = 0.002;
  int safety = 0;

  double current = Rotation.angle(turns);
  if (current >= target - tol && current <= target + tol) {
    armStop();
    return;
  }

  if (current < target) {
    armUp();
    while (Rotation.angle(turns) < target - tol && (++safety) < 2000) {
      wait(5, msec);
    }
  } else {
    armDown();
    while (Rotation.angle(turns) > target + tol && (++safety) < 2000) {
      wait(5, msec);
    }
  }

  armStop();
  */
  // while (Rotation.angle(degrees) < 320.0) {
  //   double currentAngle = Rotation.angle(degrees);

  //   if (currentAngle < 320.0) {
  //     armUp();
  //   } else if (currentAngle == 320.0)
  //   {
  //     armStop();
  //   }

  //   wait(10, msec);  // Prevent CPU overload
  // }

  // armStop();  // Final stop to ensure motor halts
  //armDown();
  // armDown(0.4);
}

//
// Main will set up the competition functions and callbacks.
//
int main() 
{
  Controller.ButtonL1.pressed(buttonPressedL1);
  Controller.ButtonR1.pressed(buttonPressedR1);
  Controller.ButtonX.pressed(buttonPressedX);
  Controller.ButtonY.pressed(buttonPressedY);
  Controller.ButtonA.pressed(buttonPressedA);
  Controller.ButtonB.pressed(buttonPressedB);
  Controller.ButtonDown.pressed(buttonPressedDown);
  Controller.ButtonL2.pressed(buttonPressedL2);
  Controller.ButtonR2.pressed(buttonPressedR2);



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
