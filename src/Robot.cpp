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

class Robot : public frc::IterativeRobot {

private:
	frc::RobotDrive myRobot{0, 1};
	frc::Joystick joystick_R {0}, joystick_L {1};
	frc::Joystick controller {2};
	frc::Talon intake {3};
	frc::Compressor *compressor = new Compressor(0);
	frc::DoubleSolenoid ramp {0, 1}, arm {2, 3}, fire {4, 5};
	frc::AnalogGyro gyro {1};
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	frc::Encoder encoder{ 0, 1, false, Encoder::EncodingType::k4X };
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
	std::stringstream strs;

	bool A(){ return controller.GetRawButton(1); }
	bool B(){ return controller.GetRawButton(2); }
	bool X(){ return controller.GetRawButton(3); }
	bool Y(){ return controller.GetRawButton(4); }
	bool Lb(){ return controller.GetRawButton(5); }  // current gyro loop break
	bool Rb(){ return controller.GetRawButton(6); }
	bool start(){ return controller.GetRawButton(7); } //find actual number

	void TurnRobot(double angle){
		double lastAngle = gyro.GetAngle();
		int speed;
		int range = 1;

		while(abs((gyro.GetAngle() - lastAngle) - angle) < range){
			speed = (angle - (gyro.GetAngle() - lastAngle)) / angle;
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

	std::string InToString(int i){
		std::stringstream ss;
		std::string s;
		ss << i ;
		ss >> s ;
		return s;
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
		//gyro.Calibrate();
	}

	void TeleopPeriodic() {
		//joystick input for the left and right drive of the robot
		myRobot.TankDrive(joystick_L.GetY(), joystick_R.GetY());

		//controller input for the intake wheels: x and y
		if (controller.GetRawButton(3)) {
			intake.Set(1);
		}
		else if (controller.GetRawButton(4)) {
			intake.Set(-1);
		}
		else {
			intake.Set(0);
		}


		//gyro things
		//convert string

		SmartDashboard::PutString("DB/String 0", InToString(gyro.GetAngle()));
		SmartDashboard::PutString("DB/String 1", InToString(gyro.GetRate()));

		//controller input for the vertical ramp controls: a and b
		if (controller.GetRawButton(1)) {
			ramp.Set(frc::DoubleSolenoid::kForward);
		}
		else if (controller.GetRawButton(2)){
			ramp.Set(frc::DoubleSolenoid::kReverse);
		}

		//controller input for the arm controls: rb and lb
		if (controller.GetRawButton(6)) {
			arm.Set(frc::DoubleSolenoid::kForward);
		}
		else if (controller.GetRawButton(5)) {
			arm.Set(frc::DoubleSolenoid::kReverse);
		}

		//controller input for the arm controls: rt
		if (controller.GetRawAxis(3) >= 0.5) {
			fire.Set(frc::DoubleSolenoid::kForward);
		}
		else {
			fire.Set(frc::DoubleSolenoid::kReverse);
		}

	}






	void TestPeriodic() {}
};

START_ROBOT_CLASS(Robot)
