/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <RobotDrive.h>
#include <Joystick.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Compressor.h>
#include <DoubleSolenoid.h>
#include <AnalogGyro.h>
#include <Talon.h>
#include <sstream>
#include <Encoder.h>
#include <Timer.h>
#include "wpilib.h"

class Robot : public frc::IterativeRobot {

private:
	frc::RobotDrive myRobot{0, 1};
	frc::Joystick joystick_R {0}, joystick_L {1};
	frc::Joystick controller {2};
	frc::Talon intake1{6}, intake2{7},
			   shooter1 {2}, shooter2{3},
			   transfer1{8}, transfer2{9};
	frc::Compressor *compressor = new Compressor(0);
	frc::DoubleSolenoid ramp {0, 1}, arm {2, 3}, fire {4, 5};
	frc::AnalogGyro gyro {1};
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	frc::Encoder encoder{ 0, 1, false, Encoder::EncodingType::k4X };
	frc::Timer timer;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
	std::stringstream strs;

	/*
	 * CONTROLLER LAYOUT, DESIGNED BY PETER
	 *
	 * LEFT BUMPER: Intake in
	 * RIGHT BUMPER: Intake out
	 *
	 * LEFT TRIGGER: Transfer to shoooter
	 * RIGHT TRIGGER: Transfer to vault
	 *
	 * X: Arms in
	 * Y: Arms out
	 *
	 * A: Move ramp up and down
	 * B: Shoot
	 */

	double shooterSpeed;

	bool A() { return controller.GetRawButton(1); }
	bool B() { return controller.GetRawButton(2); }
	bool X() { return controller.GetRawButton(3); }
	bool Y() { return controller.GetRawButton(4); }
	bool Lb() { return controller.GetRawButton(5); }  // current gyro loop break
	bool Rb() { return controller.GetRawButton(6); }
	bool Lt() { return controller.GetRawAxis(2) > 0.5; }
	bool Rt() { return controller.GetRawAxis(3) > 0.5; }
	bool Lc() { return controller.GetRawButton(9); }
	bool Rc() { return controller.GetRawButton(10); }

	bool dpad_up() {
		if ( (controller.GetPOV(0) >= 0 && controller.GetPOV(0) <= 45) || controller.GetPOV(0) == 315 ) {
			return true;
		} else {
			return false;
		}
	}

	bool dpad_down() {
		if ( controller.GetPOV(0) >= 135 && controller.GetPOV(0) <= 215 ) {
			return true;
		} else {
			return false;
		}
	}

	bool start() { return controller.GetRawButton(8); } //find actual number

	// Turns robot relative to active position
	void TurnRobot(double angle) {
		double lastAngle = gyro.GetAngle(); //angle at which robot starts
		double speed; //turning shooterSpeed
		double range = 1; // Wiggle room synonym

		while(fabs(angle - (gyro.GetAngle() - lastAngle)) > range && !Lc()){  //while not within range
			speed = (angle - (gyro.GetAngle() - lastAngle)) / angle; //sets shooterSpeed based on the angle remaining
			SmartDashboard::PutString("DB/String 2", DoubleToString(speed));
			SmartDashboard::PutString("DB/String 0", DoubleToString(gyro.GetAngle()));
			myRobot.TankDrive(1*speed, -1*speed);
		}
	}

	// TODO:Distance Tracking
		void ForwardDistance(double dist){
			encoder.Reset();
			double distLeft = dist;

			int counter = 0;
			while(encoder.GetDistance() < dist && !Lb()) {
				std::stringstream steam;
				std::string encoderValue;
				steam << encoder.GetDistance();
				steam >> encoderValue;
				SmartDashboard::PutString("DB/String 1", encoderValue);


				std::stringstream steam2;
				std::string counterValue;
				steam2 << counter;
				steam2 >> counterValue;
				SmartDashboard::PutString("DB/String 2", counterValue);

				counter++;
				distLeft = dist - encoder.GetDistance();
				myRobot.Drive(/*distLeft < 24 ? distLeft / 96 + 0.1: 00.25*/ 0.15, 0.0);
			}

			myRobot.Drive(0.0, 0.0);
		}

public:

	// Simple Type Converter
	std::string DoubleToString(double i){
		std::stringstream ss;
		std::string s;
		ss << i ;
		ss >> s ;
		return s;
	}

	void SetIntake(double value){
		intake1.Set(value);
		intake2.Set(value);
	}

	void SetTransfer(double value) {
		transfer1.Set(value);
		transfer2.Set(value);
	}

	void SetShooter(double value) {
		shooter1.Set(value);
		shooter2.Set(value);
	}

	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	}

	void AutonomousInit() override {
		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {
		//sets compressor activation
		compressor->SetClosedLoopControl(true);
		gyro.Calibrate();
	}

	void TeleopPeriodic() {
		//joystick input for the left and right drive of the robot
		myRobot.TankDrive(joystick_L.GetY(), joystick_R.GetY());

		/*
		 * LEFT BUMPER: Intake In
		 * RIGHT BUMPER: Intake Out
		 */
		if (Lb()) {
			SetIntake(-1);
		}
		else if (Rb()) {
			SetIntake(1);
		}
		else {
			SetIntake(0);
		}

		/*
		 * LEFT TRIGGER: Transfer powercube to shooter
		 * RIGHT TRIGGER: Transfer powercube to intake/vault
		 */
		if (Lt()) {
			SetTransfer(-1);
		}
		else if (Rt()) {
			SetTransfer(1);
		}
		else {
			SetTransfer(0);
		}

		/*
		 * DPAD UP: Ramp up
		 * DPAD DOWN: Ramp down
		 */
		if (dpad_up()){
			ramp.Set(frc::DoubleSolenoid::kForward);
			shooterSpeed = 1;
		} else if (dpad_down()){
			ramp.Set(frc::DoubleSolenoid::kReverse);
			shooterSpeed = 0.5;
		}

		/*
		 *  B: Turns on shooter, then fires after one second.
		 */
		if (Rt()) {
			timer.Start();
			SetShooter(shooterSpeed);
			if (timer.Get() > 1)
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

		//starts gyro turn test
		if (Rc()) {
			TurnRobot(90);
		}

		//resets gyro: start button
		if (start()){
			gyro.Reset();
		}
	}


	void TestPeriodic() {}
};

START_ROBOT_CLASS(Robot)
