/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <AnalogGyro.h>
#include <Compressor.h>
#include <DoubleSolenoid.h>
#include <Encoder.h>
#include <iostream>
#include <IterativeRobot.h>
#include <Joystick.h>
#include <LiveWindow/LiveWindow.h>
#include <RobotDrive.h>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Talon.h>
#include <Timer.h>
#include "wpilib.h"
#include "ctre/Phoenix.h"


using namespace std;

class Robot : public frc::IterativeRobot {

private:
	frc::RobotDrive myRobot{0, 1};
	frc::Joystick joystick_R {0}, joystick_L {1};
	frc::Joystick controller {2};
	frc::Talon Lintake{6}, Rintake{7},
			   shooter1 {4}, shooter2{5},
			   transfer{8};
	frc::Compressor *compressor = new Compressor(0);
	frc::DoubleSolenoid ramp {0, 1}, arm {2, 3}, fire {4, 5};
	frc::AnalogGyro gyro {1};
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	frc::Encoder encoder{ 0, 1, true, Encoder::EncodingType::k4X };
	frc::Timer timer;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
	std::stringstream strs;

	/*
	 * CONTROLLER LAYOUT, DESIGNED BY PETER
	 *
	 * LEFT BUMPER: Intake in
	 * RIGHT BUMPER: Intake out (and transfer to vault)
	 *
	 * LEFT TRIGGER: Transfer to shoooter
	 * NOPE //RIGHT TRIGGER: Transfer to vault
	 *
	 * X: Arms in
	 * Y: Arms out
	 *
	 * Dpad_Up and Dpad_Down: Move ramp up and down
	 * B: Shoot
	 */

	/*
	 * BUTTON SHORTCUTS
	 */
	bool A() { return controller.GetRawButton(1); }
	bool B() { return controller.GetRawButton(2); }  // Shooter
	bool X() { return controller.GetRawButton(3); }  // Intake arms in
	bool Y() { return controller.GetRawButton(4); }  // Intake arms out
	bool Lb() { return controller.GetRawButton(5); }  // Intake in
	bool Rb() { return controller.GetRawButton(6); }  // Intake out
	bool Lt() { return controller.GetRawAxis(2) > 0.5; }  // Transfer cube to shooter
	bool Rt() { return controller.GetRawAxis(3) > 0.5; }  // Nothing
	bool Lc() { return controller.GetRawButton(9); }  // current gyro loop break
	bool Rc() { return controller.GetRawButton(10); }
	bool dpad_up() {  // Ramp up
		return (controller.GetPOV(0) >= 0 && controller.GetPOV(0) <= 45) /*|| controller.GetPOV(0) == 315*/;
	}
	bool dpad_down() {  // Ramp down
		return controller.GetPOV(0) >= 135 && controller.GetPOV(0) <= 215;
	}
	bool dpad_left() {  // Reverse a wheel on intake
		return controller.GetPOV(0) >= 225 && controller.GetPOV(0) <= 315;
	}
	bool start() { return controller.GetRawButton(8); } // Resets the gyro in teleop
	bool R6() {return joystick_L.GetRawButton(6);}
	double LDrive() {return joystick_L.GetY();}
	double RDrive() {return joystick_R.GetY();}
	double TimerVar;
	bool shooting;
	PigeonIMU * _pidgey = new PigeonIMU(0);
	double xyz [3];


	// Turns robot relative to robot's position at the time of execution.  Delays everything until done.
	double destinationAngle, initialAngle;  bool isTurning;
	void TurnRobot2(double angle) {
		gyro.Reset();
		destinationAngle = angle;
		isTurning = true;
		initialAngle = gyro.GetAngle(); //angle at which robot starts
	}

	// Rotate robot to the right by angle WITH A WHILE LOOP for AUTO
	void TurnRobot(double angle, double scale = 90){   //takes angle in degrees, and scale in max degrees
		gyro.Reset();
		while( !(gyro.GetAngle() > (angle - 1) && gyro.GetAngle() < (angle + 1)) && !Lb() )
		{
			double angleRemaining = angle - gyro.GetAngle();
			double turnSpeed =  angleRemaining / scale + 0.1;
			myRobot.TankDrive(turnSpeed, -turnSpeed);

			SmartDashboard::PutString("DB/String 5", DoubleToString(gyro.GetAngle()));
		}



		myRobot.Drive(0.0, 0.0);
	}

	void TurnEncRobot (double angle){
		encoder.Reset();

		while (fabs(encoder.GetDistance() - angle) > 1 && !Lc()){
			SmartDashboard::PutString("DB/String 4", DoubleToString(encoder.GetDistance()));
			myRobot.Drive(0.25, 1);
		}

		myRobot.Drive(0.0, 0.0);
	}

	// TODO:Distance Tracking
		void ForwardDistance(double dist){
			encoder.Reset();
			double distLeft = dist;

			while(fabs(encoder.GetDistance() - dist) > 1 && !Lc()) {
				SmartDashboard::PutString("DB/String 4", DoubleToString(encoder.GetDistance()));
				distLeft = dist - encoder.GetDistance();
				myRobot.Drive(/*distLeft < 24 ? distLeft / 96 + 0.1: 00.25*/ dist > 0 ? -0.4 : 0.4, -0.05);
			}

			myRobot.Drive(0.0, 0.0);
		}


		// Set shooterSpeed to 1 so it will work without having to press up or down on the d-pad
		double shooterSpeed = 1;

public:
	// Simple Double To String Type Converter
	std::string DoubleToString(double i){
		std::stringstream ss;
		std::string s;
		ss << i ;
		ss >> s ;
		return s;
	}
	void SetIntake(double value){
//		Lintake.Set(value);
		Rintake.Set(dpad_left() ? -value : value);
		Lintake.Set(value);
	}
	void SetTransfer(double value) {
		transfer.Set(value);;
	}
	void SetShooter(double value) {
		shooter1.Set(value);
	shooter2.Set(value);
	}

	void RobotInit() {
		m_chooser.AddDefault("DO_NOTHING1", "DO_NOTHING2");
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
		encoder.SetSamplesToAverage(5);
		encoder.SetDistancePerPulse(1.0/360*18.84);
	}

	void AutonomousInit() override {
		shooting = false;
		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		m_autoSelected = m_chooser.GetSelected();
		m_autoSelected = SmartDashboard::GetString("Auto Selector", kAutoNameDefault);

		std::cout << "Auto selected: " << m_autoSelected << std::endl;
		SmartDashboard::PutString("DB/String 3", m_autoSelected);
/*
		TurnRobot(90, 90);

		ForwardDistance(30 * 12);
		TurnRobot(-90);

		// Ramp up
		ramp.Set(frc::DoubleSolenoid::kReverse);
		shooterSpeed = 1;
		TimerVar = 1;

		while (timer.Get() > TimerVar) {
			timer.Start();
			SetShooter(shooterSpeed);
			if (timer.Get() > TimerVar)
				fire.Set(frc::DoubleSolenoid::kForward);
		}

		timer.Stop();
		timer.Reset();
		SetShooter(0);
		fire.Set(frc::DoubleSolenoid::kReverse);
		*/
		ForwardDistance(110);
		TurnEncRobot(30);

//		ramp.Set(frc::DoubleSolenoid::kForward);
//		fire.Set(frc::DoubleSolenoid::kReverse);
//		ForwardDistance(-99);
//		shooterSpeed = 0.3;
//		shooting = true;

}

		//if(m_autoSelected[2]== 'W'){
			//sleep(1000);
		//}

    /*
	if(m_autoSelected[1] == "O"){
		if (m_autoSelected[0] == "L") {  //If our robot starts on the left
			if(gameData.length() > 0){
				if(gameData[0] == 'L'){  //If our switch is on the left
					goDistanceInches(10.5*12, 'R');
					//fire cube
					TurnRobot(-90);
					goDistanceInches(6*12, 'L');
					goDistanceInches(3.5*12, 'R');
					goDistanceInches((1.2*12)-36);
					//Pick up cube
					goDistanceInches(-6, 'R', 180);
					goDistanceInches(6);
					//fire cube
				} else if(gameData[0] == 'R'){  //If our switch is on the right
					
		if(m_autoSelected[1] == "O"){
			if (m_autoSelected[0] == "L") {  //If our robot starts on the left
				if(gameData.length() > 0){
					if(gameData[0] == 'L'){  //If our switch is on the left
						goDistanceInches(10.5*12, 'R');
						//fire cube
						TurnRobot(-90);
						goDistanceInches(6*12, 'L');
						goDistanceInches(3.5*12, 'R');
						goDistanceInches((1.2*12)-36);
						//Pick up cube
						goDistanceInches(-6, 'R', 180);
						goDistanceInches(6);
						//fire cube
					} else if(gameData[0] == 'R'){  //If our switch is on the right

					}
				}

			} else if (m_autoSelected[0] == "M") {  //If our robot starts in the middle
				if(gameData.length() > 0){
					if(gameData[0] == 'L'){  //If our switch is on the left
						goDistanceInches(3*12, 'L');
						goDistanceInches(4.5*12, 'R');
						goDistanceInches((9*12)-40);
						//fire cube
						goDistanceInches(-(6*12), 'L');
						goDistanceInches(4.5*12, 'R');
						goDistanceInches((3*12)+4);
						//pick up cube
						goDistanceInches(-0.5*12, 'R');
						goDistanceInches(4.5*12, 'R');
						goDistanceInches((4.5*12)-4);
						//fire cube
					} else if(gameData[0] == 'R'){  //If our switch is on the right
						goDistanceInches(3*12, 'R');
						goDistanceInches(4.5*12, 'L');
						goDistanceInches((9*12)-40);
						//fire cube
						goDistanceInches(-(6*12), 'R');
						goDistanceInches(4.5*12, 'L');
						goDistanceInches((3*12)+4);
						//pick up cube
						goDistanceInches(-0.5*12, 'L');
						goDistanceInches(4.5*12, 'L');
						goDistanceInches((4.5*12)-4);
						//fire cube
					}
				}
			} else if (m_autoSelected[0] == "R") {  //If our robot starts on the right
				if(gameData.length() > 0){
					if(gameData[0] == 'L'){  //If our switch is on the left

					} else if(gameData[0] == 'R'){  //If our switch is on the right
						goDistanceInches(10.5*12, 'L');
						//fire cube
						TurnRobot(-90);
						goDistanceInches(6*12, 'R');
						goDistanceInches(3.5*12, 'L');
						goDistanceInches((1.2*12)-36);
						//Pick up cube
						goDistanceInches(-6, 'L', 180);
						goDistanceInches(6);
						//fire cube
					}
				}
			}
		} else if (m_autoSelected[1] == 'D'){
			if (m_autoSelected[0] == "L") {  //If our robot starts on the left
				if(gameData.length() > 0){

					if(gameData[0] == 'L'){  //If our switch is on the left
					} else if(gameData[0] == 'R'){  //If our switch is on the right

					}
				}
			} else if (m_autoSelected[0] == "M") {  //If our robot starts in the middle
				if(gameData.length() > 0){
					if(gameData[0] == 'L'){  //If our switch is on the left

					} else if(gameData[0] == 'R'){  //If our switch is on the right

					}
				}
			} else if (m_autoSelected[0] == "R") {  //If our robot starts on the right
				if(gameData.length() > 0){
					if(gameData[0] == 'L'){  //If our switch is on the left

					} else if(gameData[0] == 'R'){  //If our switch is on the right

					}
				}
			}

		}*/

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {

		} else {
			std::string gameData;
			gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if(gameData.length() > 0) {
            	if(gameData[0] == 'L') {
            		//TurnRobot(-90);
//            		myRobot.TankDrive(-1, 1);
            	} else {
            		//TurnRobot(90);
//            		myRobot.TankDrive(1, -1);
            	}
            }
		}
	}

	void TeleopInit() {
		//sets compressor activation
		compressor->SetClosedLoopControl(true);
		encoder.Reset();
		gyro.Reset();
		_pidgey->SetYaw(0, 1000);

	}

	void TeleopPeriodic() {
		//joystick input for the left and right drive of the robot
		myRobot.TankDrive(LDrive(), RDrive());

		_pidgey->GetYawPitchRoll(xyz);

		SmartDashboard::PutString("DB/String 5", DoubleToString(gyro.GetAngle()));
		SmartDashboard::PutString("DB/String 7", DoubleToString(xyz[0]));



		if (R6()) {
			myRobot.TankDrive(-RDrive(), -LDrive());
		}

//		ledblink.Set(0.65);

		frc::SmartDashboard::PutString("DB/String 3", DoubleToString(joystick_R.GetY()));
		SmartDashboard::PutString("DB/String 4", DoubleToString(encoder.GetDistance()));


		/*
		 * LEFT BUMPER: Intake In
		 * RIGHT BUMPER: Intake Out
		 */
		if (Lb()) {
			SetIntake(-0.5);
		} else if (Rb()) {
			SetIntake(0.75);
		} else {
			SetIntake(0);
		}

		/*
		 * LEFT TRIGGER: Transfer powercube to shooter
		 * RIGHT BUMPER: Transfer powercube to intake/vault
		 */
		if (Lt()) {
			SetTransfer(0.6);
		} else if (Rb()) {
			SetTransfer(-0.6);
		} else {
			SetTransfer(0);
		}


		/*
		 * DPAD UP: Ramp up
		 * DPAD DOWN: Ramp down
		 */
		if (dpad_up()){
			ramp.Set(frc::DoubleSolenoid::kReverse);
			shooterSpeed = 1;
			TimerVar = 1;
		}
		else if (dpad_down()){
			ramp.Set(frc::DoubleSolenoid::kForward);
			shooterSpeed = 0.30;
			TimerVar = 0.25;
		}

		/*
		 *  B: Turns on shooter, then fires after one second.
		 */
		if (B()) {
			timer.Start();
			SetShooter(shooterSpeed);
			if (timer.Get() > TimerVar)
				fire.Set(frc::DoubleSolenoid::kForward);
		} else {
			timer.Stop();
			timer.Reset();
			SetShooter(0);
			fire.Set(frc::DoubleSolenoid::kReverse);
		}

		/*
		 * X: Brings arms in
		 * Y: Brings arms out
		 */
		if (X()) {
			arm.Set(frc::DoubleSolenoid::kReverse);
		}
		else if (Y()) {
			arm.Set(frc::DoubleSolenoid::kForward);
		}

		SmartDashboard::PutString("DB/String 0", DoubleToString(gyro.GetAngle()));
		SmartDashboard::PutString("DB/String 1", DoubleToString(gyro.GetRate()));

		if (Rc())
			isTurning=false;

		//resets gyro: start button
		if (start()){
			gyro.Reset();
		}

/*		if (isTurning) {
			double lastAngle = gyro.GetAngle(); //angle at which robot starts
			double speed; //turning shooterSpeed
			double range = 1; // Wiggle room

			if(fabs(destinationAngle - (gyro.GetAngle() - lastAngle)) > range && !Lc()) {  //while not within range
				speed = (destinationAngle - (gyro.GetAngle() - lastAngle)) / fabs(destinationAngle); //sets shooterSpeed based on the angle remaining
				SmartDashboard::PutString("DB/String 2", DoubleToString(speed));
				SmartDashboard::PutString("DB/String 0", DoubleToString(gyro.GetAngle()));
				myRobot.TankDrive(1*speed, -1*speed);
			} else
				isTurning = false;
		}*/
	}


	void TestPeriodic() {}
};

START_ROBOT_CLASS(Robot)
