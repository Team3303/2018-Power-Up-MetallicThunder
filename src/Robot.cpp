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

class Robot : public frc::IterativeRobot {

private:
	//this is robot drive my robot 0 and 1 semicolon
	frc::RobotDrive myRobot{0 , 1};
	frc::Joystick joystick_R {0}, joystick_L {1};
	frc::Joystick controller {2};
	frc::Compressor *compressor = new Compressor(0);
	frc::DoubleSolenoid rampSolenoid {0, 1};
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;

public:
	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */


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

		//controller input for the vertical ramp controls
		if (controller.GetRawButton(1)) {
			rampSolenoid.Set(frc::DoubleSolenoid::kForward);
		}
		else if (controller.GetRawButton(2)){
			rampSolenoid.Set(frc::DoubleSolenoid::kForward);
		}

	}

	void TestPeriodic() {}
};

START_ROBOT_CLASS(Robot)
