#include "WPILib.h"

class Robot: public SampleRobot
{
	RobotDrive *myRobot;
	Joystick *rightJoy;
	Joystick *leftJoy;

	Victor *lfWheel; //left front wheel
	Victor *lrWheel; //left rear wheel
	Victor *rfWheel; //right front wheel
	Victor *rrWheel; //right rear wheel

public:
	Robot() {
		//not sure what the ports are
		myRobot = new RobotDrive(0,1);
		rightJoy = new Joystick(1);
		leftJoy = new Joystick(2);

		lfWheel = new Victor(14);
		lrWheel = new Victor(15);
		rfWheel = new Victor(0);
		rrWheel = new Victor(1);
	}

	void RobotInit()
	{

	}

	void Autonomous()
	{
		myRobot->SetSafetyEnabled(false);
	}

	void OperatorControl()
	{
		double rightJoyX;
		double leftJoyY;

		myRobot->SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled()) {
			leftJoyY = leftJoy->GetY();
			rightJoyX = rightJoy->GetX();

			lrWheel->SetSpeed(leftJoyY + rightJoyX);
			lfWheel->SetSpeed(leftJoyY + rightJoyX);
			rfWheel->SetSpeed(leftJoyY - rightJoyX);
			rrWheel->SetSpeed(leftJoyY - rightJoyX);
		}

		lfWheel->SetSpeed(0);
		lrWheel->SetSpeed(0);
		rfWheel->SetSpeed(0);
		rrWheel->SetSpeed(0);
	}

	/**
	 * Runs during test mode
	 */
	void Test()
	{
	}
};

START_ROBOT_CLASS(Robot)
