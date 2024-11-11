#include "vex.h"
#include "odometry.h"


#include "vex.h"
using namespace vex;
using signature = vision::signature;
using code = vision::code;

void  vexcodeInit( void );
// A global instance of competition
competition Competition;

//robot config.cpp stuff
motor LF = motor(PORT8, ratio18_1, true);
motor LM = motor(PORT10, ratio18_1, true);
motor LB = motor(PORT2, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(LF, LM, LB);
motor RF = motor(PORT3, ratio18_1, false);
motor RM = motor(PORT15, ratio18_1, false);
motor RB = motor(PORT1, ratio18_1, false);
motor_group RightDriveSmart = motor_group(RF, RM, RB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
inertial Inertial1 = inertial(PORT7);
rotation Front_tracking = rotation(PORT20);
rotation Side_tracking = rotation(PORT19);
rotation claw_tracking = rotation(PORT18);
rotation intake_tracking = rotation(PORT4);
optical Optical1 = optical(PORT17);

brain  Brain;
controller Controller1 = controller(primary);
motor intake = motor(PORT9, ratio6_1, false);
motor claw_mech = motor(PORT6, ratio6_1,false);
// Pistons
digital_out Clamp = digital_out(Brain.ThreeWirePort.H);
digital_out PTO = digital_out(Brain.ThreeWirePort.B);
digital_out descore_mech = digital_out(Brain.ThreeWirePort.E);
digital_out elevation = digital_out(Brain.ThreeWirePort.F);


void blue_detected() {
  if (Optical1.color() == blue){
    intake.spin(reverse);
  }
  else if (Optical1.color() == ClrSkyBlue){
    intake.spin(reverse);
  }
  else if (Optical1.color() == ClrLightBlue){
    intake.spin(reverse);
  }
}
void red_detected() {
  if (Optical1.color() == red){
    intake.spin(reverse);
  }
}


void funny_prank() {
  Optical1.setLightPower(100,pct);
  Optical1.objectDetected(blue_detected);
  Optical1.objectDetected(red_detected);
  wait (3,sec);
}
      

void PID_turn(double LeftVelocity,double RightVelocity,double inches_traveled) {
  LeftDriveSmart.setVelocity(LeftVelocity, pct);
  RightDriveSmart.setVelocity(RightVelocity,pct);
  Drivetrain.driveFor(inches_traveled, inches,false);
  wait(1,sec);
  Drivetrain.stop();
}

void Reset_Both_Sides(double same_velocity) {
  LeftDriveSmart.setVelocity(same_velocity, pct);
  RightDriveSmart.setVelocity(same_velocity,pct);
}

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  bool Controller1RightShoulderControlMotorsStopped = true;
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
    claw_tracking.setPosition(10,degrees);
      if (Controller1.ButtonR1.pressing()) {
        while (claw_tracking.position(degrees) <= 70) {
          claw_mech.spin(forward);
      }
      wait (.1,sec);
      claw_mech.spin(forward);
      claw_mech.stop(hold);
      Controller1RightShoulderControlMotorsStopped = false;
      }
       else if (Controller1.ButtonR2.pressing()) {
        claw_mech.spin(reverse);
        Controller1RightShoulderControlMotorsStopped = false;

      } else if (!Controller1RightShoulderControlMotorsStopped) {
        claw_mech.stop(hold);
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }

      int drivetrainLeftSideSpeed = Controller1.Axis3.position();
      int drivetrainRightSideSpeed = Controller1.Axis2.position();

      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}


void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}


//robot config.cpp stuff



bool Clamping = false;
bool elevate = false;
bool tier_1 = false;

//settings:
double kp = 0.0;
double ki = 0.0;
double kd = 0.0;
double turnkp = 0.0;
double turnki = 0.0;
double turnkd = 0.0;

//autonomous settings
int desiredValue = 200;
int desiredTurnValue = 0;

int Error; 
//sensor value - desired value : position
int PrevError = 0; 
//position 20 miliseconds ago
int Derivative; 
//Error - PrevError : speed
int TotalError; 
//TotalError = TotalError + Error

int turnError;
int turnPrevError = 0;
int turnDerivative;
int turnTotalError;

bool ResetDriveSensors = true;

//varibles modified for use:
bool enablePID_Drive = true;
int PID_Drive (){

while (enablePID_Drive) {
  if (ResetDriveSensors) {
    ResetDriveSensors = false;
    LeftDriveSmart.setPosition(0,degrees);
    RightDriveSmart.setPosition(0,degrees);
  }

  //get the position of both motors
  int leftMotorPosition = LeftDriveSmart.position(degrees);
  int rightMotorPosition = RightDriveSmart.position(degrees);
  

  ////////////////////////////////////////////////////////////////////////////////
  //lateral movement PID
  ///////////////////////////////////////////////////////////////////////////////

  //get average of the two motors
  int averagePosition = (leftMotorPosition + rightMotorPosition)/2;

    //potential
    Error = averagePosition - desiredValue;

    //derivative
    Error = Error - PrevError;

    //velocity -> position -> absement
    TotalError += Error;

    //integral
    TotalError += Error;

    double lateralMotorPower = Error * kp + Derivative * kd + TotalError * ki;

   ////////////////////////////////////////////////////////////////////////////////



  ////////////////////////////////////////////////////////////////////////////////
  //turning movement PID
  ///////////////////////////////////////////////////////////////////////////////
  //get average of the two motors
  int turnDifference = (leftMotorPosition + rightMotorPosition)/2;

    //potential
    turnError = turnDifference - desiredTurnValue;

    //derivative
    turnDerivative = turnError - turnPrevError;

    //velocity -> position -> absement
    TotalError += Error;

    //integral
    //turnTotalError += turnError

    double turnMotorPower = turnError * turnkp + turnDerivative * turnkd;


  ////////////////////////////////////////////////////////////////////////////////

    LeftDriveSmart.spin(forward, lateralMotorPower + turnMotorPower, velocityUnits::pct);
    RightDriveSmart.spin(forward, lateralMotorPower + turnMotorPower, velocityUnits::pct);





  PrevError = Error;
  turnPrevError = Error;
}

return 1;
}



void pre_auton (void) {
  vexcodeInit();
  Brain.Screen.setCursor(6,7);
  Brain.Screen.print(globalXPos);
  Brain.Screen.setCursor(9,7);
  Brain.Screen.print(globalYPos);
}
void autonomous(void) {
  
  /*vex::task DrivetrainPID(PID_Drive);

  desiredValue = 300;
  desiredTurnValue = 600;

  vex::task::sleep(1000);

  ResetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 300;*/
} 

bool notstuck = true;
void usercontrol(void) {
  //enablePID_Drive = false;
  // User control code here, inside the loop
  while (1) {
    claw_mech.setVelocity(100, pct);
    intake.setVelocity(100, pct);
    Drivetrain.setDriveVelocity(100, pct);
    if (Controller1.ButtonUp.pressing()) {
      notstuck = true;
      if (notstuck == true) {
      claw_mech.setVelocity(100, pct);
      while (claw_tracking.position(degrees) >= 10){
            claw_mech.spin(reverse);
          }
          wait (.1,sec);
          claw_mech.spinFor(140,degrees);
          claw_mech.stop(hold);
      }
    }
    if (Controller1.ButtonRight.pressing()) {
        notstuck = false;
        wait (.1,sec);
        claw_mech.spinFor(60,degrees);
        claw_mech.stop(hold);
    }
    // Clamp toggle
    if (Controller1.ButtonY.pressing()) {
      if (Clamping == false){
        Clamping = true;
        wait(.1, sec);        
      }
       else if (Clamping == true){
        Clamping = false;
        wait(.1, sec);
      }
    }
    Clamp.set(Clamping);

    // Descore mech toggle    
  if (Controller1.ButtonB.pressing()) {
      if (elevate == false){
        elevate = true;
        wait(.1, sec);
      }
       else if (elevate == true){
        elevate = false;
        wait(.1, sec);
      }
    }
    descore_mech.set(elevate);

    // Elevation toggle
  if (Controller1.ButtonDown.pressing()) {
      if (tier_1 == false){
        claw_mech.setVelocity(100,pct);
        while (claw_tracking.position(degrees) >= 10){
            claw_mech.spin(reverse);
          }
          wait (.1,sec);
          claw_mech.spinFor(240,degrees);
          claw_mech.stop(hold);
          elevation.set(true);
          claw_mech.spinFor(-140,degrees);
          claw_mech.stop(hold);
          tier_1 = true;
      }
       else if (tier_1 == true){
        tier_1 = false;
        wait(.1, sec);
      }
    }
    elevation.set(tier_1);
    // Intake Spinner
    if (Controller1.ButtonL1.pressing()) {
        intake.spin(forward);
        // insert function here
    }else if (Controller1.ButtonL2.pressing()) {
      intake.spin(reverse);
    }else if (true){
      intake.stop();
    }
  LeftDriveSmart.setVelocity(100, pct);
  RightDriveSmart.setVelocity(100,pct);
  
  
  
    wait(.1, sec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
   }
}



//kasen did not miss a semicolon - mckay

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  //task odometryTask = task(positionTracking);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}