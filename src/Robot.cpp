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

class Robot : public frc::IterativeRobot {

private:
	//this is robot drive my robot 0 and 1 semicolon
	frc::RobotDrive myRobot{0, 1};
	frc::Joystick joystick_R {0}, joystick_L {1};
	frc::Joystick controller {2};
	frc::Talon intake {3};
	frc::Compressor *compressor = new Compressor(0);
	frc::DoubleSolenoid ramp {0, 1}, arm {2, 3}, fire {4, 5};
	frc::AnalogGyro gyro {1};
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
	std::stringstream strs;

public:
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
		strs << gyro.GetAngle();
		std::string str;
		strs >> str;
		SmartDashboard::PutString("DB/String 0", str);

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
