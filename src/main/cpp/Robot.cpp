/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
// 3839 GitHub,Celtics,Pelicans, and Queen Bronny

// ROBOT INCLUDES ----------------------------------------
#include "Robot.h"
#include "frc/Preferences.h"
#include "frc/WPILib.h"
#include "math.h"
#include <string>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/I2C.h>


// COMPONENT INCLUDES ------------------------------------
#include "Components/MyMathRoutines.h"
#include "Components/NavX/AHRS.h"
#include "Components/Motors.h"
#include "Components/Controller_2Axis.h"		// Joystick + Rotary Knob for Driving
#include "Components/Pneumatics.h"
#include "Components/IRSensor.h"
#include "Components/Lidar.h"
AHRS *navxGyro;
AHRS *navxGyroUSB;
frc::AnalogInput distanceFrontSensor{0};
frc::AnalogInput distanceRightSensor{1};

frc::Preferences *prefs;

// SYSTEM INCLUDES ---------------------------------------
float X0, Y0, W0;
int ShootRPM = 4000;	
int AutonShootRPM = 3800;

const static int ShootMotorHiRPM = 4400;
const static int ShootMotorLoRPM = 1000;
const static int ShootMotorMidRPM = 3700;
int ShootMotorDirection = 1;


int DriveMode = 0;		// 0 -> Driver with 2-Axis Joystick and Knob, 1 - Driver with 3-Axis Joystick

int  gs_path = 1;
int  gs_color = 1;		// gs_path = 1 for PATH A, 2 for PATH B; gs_color = 1 for RED, 2 for BLUE

// COMPUTER VISION ---------------------------------------
float pc_err;			// cross-car error for powercell
float pc_fwd;			// forward speed when picking up powercell
float pc_strf;			// strafe speed when picking up powercell
float pc_area;			// area of detected powercell

#include "Systems/SwerveDrive.h"
#include "Systems/Pickup.h"
#include "Systems/ControlPanel.h"
#include "Systems/Accumulator.h"
#include "Systems/Shooter.h"
#include "Systems/Climber.h"
#include "Systems/AutonRoutines.h"
#include "Systems/GalacticSearch.h"
#include "Systems/AutoNav_Slalom.h"
#include "Systems/AutoNav_Barrel.h"
#include "Systems/AutoNav_Bounce.h"

void Robot::RobotInit() {

	// m_chooser.SetDefaultOption("Auton 0 : Do Nothing", kAutoNameDefault);
	// m_chooser.AddOption("Auton 1 : Move Fwd", kAuto1NameCustom);
	// m_chooser.AddOption("Auton 2 : Shoot + Move Fwd", kAuto2NameCustom);
	// m_chooser.AddOption("Auton 3 : Shoot + Move Back", kAuto3NameCustom);
	// m_chooser.AddOption("Auton 4 : Shoot + Extras", kAuto4NameCustom);
	// m_chooser.AddOption("Auton 5 : Shoot From Middle + Move Back", kAuto5NameCustom);
	// frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
	// navX Gyroscope
	//navxGyro = new AHRS(SerialPort::Port::kUSB);
	//navxGyro = new AHRS(SPI::Port::kMXP);
	//navxGyro->ZeroYaw();

	navxGyro = new AHRS(SerialPort::Port::kUSB, AHRS::SerialDataType::kProcessedData, 200);
	navxGyro->ZeroYaw();
	//Alternative Ports:  I2C::Port::kMXP, SerialPort::Port::kMXP or SPI::Port::kMXP 
	
	// SET UP COMPONENTS
	SetUpControllers();
	SetUpPneumatics();
	SetupIRSensors();  

	// LIDAR - Benewake TF-Luna on Arduino Uno	
	SetupLidarSensors();	
	
	// Analog MaxBotix Ultrasonic Sensors
	distanceFrontSensor.SetOversampleBits(4);
	distanceRightSensor.SetOversampleBits(4);
	
	distanceFrontSensor.SetAverageBits(4);
	distanceRightSensor.SetAverageBits(4);

	// Galactic Search Autonomous Move Parameters
	frc::SmartDashboard::PutNumber("PATH", gs_path);
	frc::SmartDashboard::PutNumber("COLOR", gs_color);

	// Robot preferences
	//prefs = frc::Preferences::GetInstance();

	// Swerve Drive PID Gains - Pickup Arm Up
	frc::SmartDashboard::PutNumber("driveP", 0.7);
	frc::SmartDashboard::PutNumber("driveI", 0);
	frc::SmartDashboard::PutNumber("driveD", 0.08);
	frc::SmartDashboard::PutNumber("deltaT", 0.005);
	
	// Drive Mode
	frc::SmartDashboard::PutNumber("Drive_Mode", DriveMode);

	// How hard to hold on to steering angle

	frc::SmartDashboard::PutNumber("Steering_Hold", 0.0005);
		
	SetupMotors();

	CellIntakeStatusCurrent = CellIntakeSensor->Get();
	frc::SmartDashboard::PutBoolean("Cell Intake Sensor", CellIntakeStatusCurrent);
	
	// SET UP INITIAL POSITION OF SYSTEMS
	pickupArmPosition = pickupDownSolenoidPosition;
	SetPickupArmPosition(pickupUpSolenoidPosition);

	SetClimberPosition(climberDownSolenoidPosition);
	//isClimberMotorEnabled = false;
	//SetPickupMotors(pickupMotorStop);

	// GET JOYSTICK ZEROES
	if (DriveMode == 0)		// If driving with joystick and knob
	{
		X0 = translateJoystick->GetX();  // Side-to-side speed
		Y0 = translateJoystick->GetY();  // Fore-aft speed
		W0 = rotateJoystick->GetX(); // Rotation speed 
	}
	else
	{
		X0 = translateJoystick->GetX();  // Side-to-side speed
    	Y0 = translateJoystick->GetY();  // Fore-aft speed
    	W0 = translateJoystick->GetZ();  // Rotation speed 
	}	

	prefs = frc::Preferences::GetInstance();	

	// AutoNav Slalom loopcounts
	frc::SmartDashboard::PutNumber("slalom1", 330);	
	frc::SmartDashboard::PutNumber("slalom2", 1500);	
	frc::SmartDashboard::PutNumber("slalom3", 360);	
	frc::SmartDashboard::PutNumber("slalom4", 400);	
	frc::SmartDashboard::PutNumber("slalom5", 360);	
	frc::SmartDashboard::PutNumber("slalom6", 400);	
	frc::SmartDashboard::PutNumber("slalom7", 360);	
	frc::SmartDashboard::PutNumber("slalom8", 1500);	
	frc::SmartDashboard::PutNumber("slalom9", 370);	
	frc::SmartDashboard::PutNumber("slalom10", 200);	

	frc::SmartDashboard::PutBoolean("Left Lidar",CheckLeftLidar());
	frc::SmartDashboard::PutBoolean("Right Lidar",CheckRightLidar());	
	
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
	
  // Check chosen Autonomous Mode
  //m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  //std::cout << "Auto selected: " << m_autoSelected << std::endl;
  
//   // Check chosen Auton start position
//   m_autonStartPositionSelected = m_autonStartPositionChooser.GetSelected();
  
//   // Check chosen Wheel Zero-ing Mode
//   m_zeroWheelsSelected = m_zeroWheelsChooser.GetSelected();

// Check RPM for first shot (with preloaded power cells)
//AutonShootRPM = frc::SmartDashboard::GetNumber("Auton Shoot 1 RPM", AutonShootRPM);

}

void Robot::AutonomousPeriodic() {

	// // We are starting auton using ma'homies technology, so no automatic zeroing
	// navxGyro->ZeroYaw();    
	// FaceAngle = navxGyro->GetYaw()*deg2rad;
	// SetSteerMotorInitialPosition();
		
	// // -------------------------------------------------------------------------------------
	// // START "AUTON" AUTON
	// // -------------------------------------------------------------------------------------
	// if (m_autoSelected == kAuto1NameCustom) 
	// { 
	// 	Auton1();
	// } 
	// else if (m_autoSelected == kAuto2NameCustom) 
	// { 
	// 	Auton2();	
	// }
	// else if (m_autoSelected == kAuto3NameCustom) 
	// { 
	// 	Auton3();
	// }
	// else if (m_autoSelected == kAuto4NameCustom) 
	// { 
	// 	Auton4();
	// }
	// else //-----------------------------------------------------------------------------
	// { 
	// 	// Default Auto goes here : don't move
	// 	frc::SmartDashboard::PutString("Auton executed", "0");
	// }
	navxGyro->ZeroYaw();    
	FaceAngle = navxGyro->GetYaw()*deg2rad;
	SetSteerMotorInitialPosition();
	SetMotorsToBrake();
	//Slalom_New();
	//Barrel_New2();
	//Bounce_New();
	GS_A_Red();
	//GS_B_Red();
	SetMotorsToCoast();
	
	while (IsAutonomous());
}

void Robot::TeleopInit()
{
	shootMotorPIDController.SetReference(0, rev::ControlType::kVoltage);
	isShooterMotorSpinning = false;
	accumulatorMotor1->Set(0);		// vertical power cell motor
	accumulatorMotor2->Set(0);		// horizontal power cell motor  
	pickupMotor->Set(0);
	SetPickupArmPosition(pickupUpSolenoidPosition);
	ShootRPM = frc::SmartDashboard::GetNumber("Desired ShootRPM", 4000);

	DriveMode = frc::SmartDashboard::GetNumber("Drive_Mode", 0);
	
	ZeroWheelsOnly();
	navxGyro->ZeroYaw();    
	FaceAngle = navxGyro->GetYaw()*deg2rad;
	SetSteerMotorInitialPosition();
}

void Robot::TeleopPeriodic() 
{

  float X, Y, W, A;  
  int t, shootcounter, swervedrivezerocounter; 

  // GALACTIC SEARCH VARIABLES
  float strafe, forward_search, forward, area2pickup, align, waitaftercomplete;

	// steerMotor1.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, kTimeoutMs);
	// steerMotor2.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, kTimeoutMs);
	// steerMotor3.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, kTimeoutMs);
	// steerMotor0.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, kTimeoutMs);

  while (IsOperatorControl() && IsEnabled())
  {

	frc::SmartDashboard::PutBoolean("Cell Intake Sensor", CellIntakeSensor->Get());

	// Lidar sensors will be "false" when no objects within 15 inches in front of them
	// and truee when there are objects within 15 inches in front of sensor
	frc::SmartDashboard::PutBoolean("Left Lidar",CheckLeftLidar());
	frc::SmartDashboard::PutBoolean("Right Lidar",CheckRightLidar());

	if (DriveMode == 0)
	{
		X = translateJoystick->GetX() - X0;  // Side-to-side speed
    	Y = translateJoystick->GetY() - Y0;  // Fore-aft speed
    	W = rotateJoystick->GetX() - W0;
	}

	if (DriveMode != 0)
	{
		X = translateJoystick->GetX() - X0;  // Side-to-side speed
    	Y = translateJoystick->GetY() - Y0;  // Fore-aft speed
    	W = translateJoystick->GetZ() - W0;  // Rotation speed 
	}

	//X=0;Y=0;W=0;
	
    A = navxGyro->GetYaw();  	
	frc::SmartDashboard::PutNumber("Yaw", A);
	frc::SmartDashboard::PutNumber("Face", FaceAngle);
	
	//Read PID gains from SmartDashboard
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.085);		// change this to 0.7 if body oscillates
	gainI = frc::SmartDashboard::GetNumber("driveI", 0);
	gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

	// Calculate Swerve Drive Parameters
	SwerveDriveMe(Y*0.8, -X*0.8, -W*0.4, A, false, 0, gainP, gainI, gainD);

	// Move
	Locomotion(TeleopDrive);    

	//---------------------------------------------------------------------------------------------------------------------------------------------
	// // PICKUP ARM TOGGLE
	if(coDriverJoystick->GetRawButton(pickupArmUpButton)) 
	{
		while(coDriverJoystick->GetRawButton(pickupArmUpButton));		
		SetPickupArmPosition(pickupUpSolenoidPosition);
	}

	if(coDriverJoystick->GetRawButton(pickupArmDownButton)) 
	{
		while(coDriverJoystick->GetRawButton(pickupArmDownButton));	
		SetPickupArmPosition(pickupDownSolenoidPosition);
	}
	// //---------------------------------------------------------------------------------------------------------------------------------------------
	// PICKUP MOTORS
	if(coDriverJoystick->GetRawButton(pickupMotorIntakeButton))   // if pickup intake motors button is held
	{
		SetPickupMotors(pickupMotorIntake);
	}
	else if(coDriverJoystick->GetRawButton(pickupMotorReverseButton))   // if pickup intake motors button is held
	{
		SetPickupMotors(pickupMotorReverse);
	}
	else
	{
		SetPickupMotors(pickupMotorStop);
	}	
	//---------------------------------------------------------------------------------------------------------------------------------------------
	// SHOOTER
	
	// Display current shoot motor RPM
	frc::SmartDashboard::PutNumber("Shoot Motor Current RPM", shootMotorEncoder.GetVelocity());
	frc::SmartDashboard::PutNumber("Shooter RPM Joystick", coDriverJoystick->GetRawAxis(3));

	if(coDriverJoystick->GetRawButton(5))
	{
		while(coDriverJoystick->GetRawButton(5));
		ShootMotorDirection = 1;		// Shoot out
	}

	if(coDriverJoystick->GetRawButton(6))
	{
		while(coDriverJoystick->GetRawButton(5));
		ShootMotorDirection = -1;		// Reverse Direction
	}

	if (shootMotorEncoder.GetVelocity() > 10 || shootMotorEncoder.GetVelocity() < -10)
	{
		frc::SmartDashboard::PutString("Shoot Motor Status", "ON");
	}
	else
	{
		frc::SmartDashboard::PutString("Shoot Motor Status", "OFF");
	}
	
	//-------------------------------------------------------------------------
	if (coDriverJoystick->GetRawButton(shootMotorONButton))	// spin up shooter motor
	{			
		while(coDriverJoystick->GetRawButton(shootMotorONButton));	// motor will spin as soon as button is released

		//-----------------------------------------------------
		// this is to experiment with shooter Motor PID loop
		// comment out once we found optimum control gain parameters
		// ShootMotorGainP = frc::SmartDashboard::GetNumber("Shoot Motor Gain P", 6e-5); 
		// ShootMotorGainI = frc::SmartDashboard::GetNumber("Shoot Motor Gain I", 1e-6);
		// ShootMotorGainD = frc::SmartDashboard::GetNumber("Shoot Motor Gain D", 0);

		// shootMotorPIDController.SetP(ShootMotorGainP);
		// shootMotorPIDController.SetI(ShootMotorGainI);
		// shootMotorPIDController.SetD(ShootMotorGainD);
		//-----------------------------------------------------
		shootMotorPIDController.SetReference(ShootRPM, rev::ControlType::kVelocity);
		isShooterMotorSpinning = true;
	}

	if (coDriverJoystick->GetRawButton(shootMotorOFFButton))	// stop motor
	{
		while(coDriverJoystick->GetRawButton(shootMotorOFFButton)); // motor will stop as soon as button is released

		ShootRPM = 0;
		shootMotorPIDController.SetReference(ShootRPM, rev::ControlType::kVoltage);
		isShooterMotorSpinning = false;		
		frc::SmartDashboard::PutString("Shoot Motor Status", "OFF");
	}
	//-------------------------------------------------------------------------
	if (isShooterMotorSpinning)
	{
		if (coDriverJoystick->GetRawAxis(3) < -0.8)
		{
			//ShootRPM = frc::SmartDashboard::GetNumber("Desired ShootRPM", 0);
			ShootRPM = ShootMotorHiRPM;
		}
		else
		{
			ShootRPM = ShootMotorMidRPM;

			if (coDriverJoystick->GetRawAxis(3) > 0.8)
			{
				//ShootRPM = frc::SmartDashboard::GetNumber("Desired ShootRPM", 0);
				ShootRPM = ShootMotorLoRPM*ShootMotorDirection;
			}
		}

		shootMotorPIDController.SetReference(ShootRPM, rev::ControlType::kVelocity);
		isShooterMotorSpinning = true;		
	}
	else
	{
		ShootRPM = 0;
		shootMotorPIDController.SetReference(ShootRPM, rev::ControlType::kVoltage);
		isShooterMotorSpinning = false;			
		frc::SmartDashboard::PutString("Shoot Motor Status", "OFF");
	}
	//-------------------------------------------------------------------------	
	if (coDriverJoystick->GetRawButton(shootButton))   // if shoot button is held
	{
		if (isShooterMotorSpinning == true)
		{
			accumulatorMotor1->Set(0.5*accumulatorMotor1Direction);		// vertical power cell motor
			accumulatorMotor2->Set(accumulatorMotor2Direction);			// horizontal power cell motor				
		}
		else
		{
			accumulatorMotor1->Set(0);
			accumulatorMotor2->Set(0);	
		}				
	}
	else
	{
		if (!coDriverJoystick->GetRawButton(pickupMotorReverseButton))
		{
			accumulatorMotor1->Set(0);
			accumulatorMotor2->Set(0);
		}
	}
	
	//---------------------------------------------------------------------------------------------------------------------------------------------
	// ZERO-ING SWERVE DRIVE
	if(translateJoystick->GetRawButton(zeroSwerveDriveButton))
	{
		while(translateJoystick->GetRawButton(7));		
		ZeroWheelsOnly();
	}
	//---------------------------------------------------------------------------------------------------------------------------------------------
	// ROTATING SMALL ANGLES
	if(translateJoystick->GetRawButton(4)) // rotate a small angle cw ---------------------
	{
		while(translateJoystick->GetRawButton(4));
		AutonMove(0, 0, 0.6, false, 0, 50);		
	}
	
	if(translateJoystick->GetRawButton(5)) // rotate a small angle ccw --------------------
	{
		while(translateJoystick->GetRawButton(5));
		AutonMove(0, 0, -0.6, false, 0, 50);		
	}
	//---------------------------------------------------------------------------------------------------------------------------------------------

	// Re-zero and Re-Orient everything
	if(translateJoystick->GetRawButton(7))
	{
		while(translateJoystick->GetRawButton(7));
	 	ZeroWheelsOnly();
	 	navxGyro->ZeroYaw();  
		 
	}

	
	// Driver Button 6 to test Auton
	if(translateJoystick->GetRawButton(6))
	{
		while(translateJoystick->GetRawButton(6));
		SetMotorsToBrake();			

		// Auton Code Here
		//GS_B_Red();
		//Slalom_New2();
		Barrel_New2();

		SetMotorsToCoast();
	}

	// // Galactic Search Test
	// if(translateJoystick->GetRawButton(9))
	// {
	// 	while(translateJoystick->GetRawButton(9));

		
	// 	// Put steering angle at zero positions and wheel speed at zero
	// 	for(t=0;t<4;t++)
	// 	{
	// 		wheelAngle[t]=0;
	// 		throttle[0] = 0;
	// 	}

	// 	Locomotion(AutonomousDrive);
	// 	frc::Wait(0.5);

	// 	navxGyro->ZeroYaw();

	// 	// Call Move 1
	// 	gs0_fwd = 0;
	// 	gs0_strf = 0;
	// 	gs3_fwd = 0;
	// 	gs3_strf = -0.3;  // -ve move left
	// 	gs3_loopcount = 500;
	// 	GSMove(gs0_fwd, gs0_strf, gs3_fwd, gs3_strf, gs3_loopcount);
	// 	GS_ReturnToZeroYaw();	

	// 	// Call Move 2
	// 	gs0_fwd = 0;
	// 	gs0_strf = -0.2;
	// 	gs3_fwd = 0;
	// 	gs3_strf = 0.3;  // -ve move left
	// 	gs3_loopcount = 500;
	// 	GSMove(gs0_fwd, gs0_strf, gs3_fwd, gs3_strf, gs3_loopcount);
	// 	GS_ReturnToZeroYaw();
	//}

	//while (true);

	
	// CLIMBER
	// ClimberFunctions();

	// if (translateJoystick->GetRawButton(11))	
	// {
	// 	climberMotorPIDController.SetReference(2000, rev::ControlType::kVelocity);
	// } 	
	// else
	// {
	// 	climberMotorPIDController.SetReference(0, rev::ControlType::kVoltage);
	// }	
	//---------------------------------------------------------------------------------------------------------------------------------------------
	// TEST AUTON MOVES TO GET THE CORRECT PID GAINS FOR DRIVING STRAIGHT
	// COMMENT OUT AFTER DONE
	// if(translateJoystick->GetRawButton(3))
	// {
	// 	while(translateJoystick->GetRawButton(3));

	// 	//AutonMove(Cross-Car, Fore-Aft, Rotation, etc....)
	// 	//	Cross-Car -> left is negative, right is positive
	// 	//	Fore-Aft -> backward is negative, forward is positive
	// 	AutonMove(0, 0, 0.6, false, 0, 200);
	// 	AutonMove(0, 0, 0, false, 0, 1);

	// 	// AutonMove(frc::SmartDashboard::GetNumber("Auton X", 0), 
	// 	// 		  frc::SmartDashboard::GetNumber("Auton Y", 0),
	// 	// 		  frc::SmartDashboard::GetNumber("Auton W", 0), 
	// 	// 		  frc::SmartDashboard::GetBoolean("Auton isSetFaceAngle", false), 
	// 	// 		  frc::SmartDashboard::GetNumber("Auton faceAngleP", 0), 
	// 	// 		  frc::SmartDashboard::GetNumber("Auton loopCount", 0));

	// 	///AutonMove(0, 0, 0, frc::SmartDashboard::GetBoolean("Auton isSetFaceAngle", false), frc::SmartDashboard::GetNumber("Auton faceAngleP", 0), 1000);
	// }
	//---------------------------------------------------------------------------------------------------------------------------------------------
    dt = frc::SmartDashboard::GetNumber("deltaT", 0.005);
	frc::Wait(dt);     // Slight delay, adjust as necessary
	
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
