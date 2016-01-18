#include "WPILib.h"

class Robot: public IterativeRobot
{

private:
	LiveWindow *lw = LiveWindow::GetInstance();
	Joystick *rightJoy;
	Joystick *leftJoy;
	Joystick *operatorJoy;
	Joystick *intakeJoy;

	Talon *lfWheel; //left front wheel
	Talon *lrWheel; //left rear wheel
	Talon *rfWheel; //right front wheel
	Talon *rrWheel; //right rear wheel
	Talon *armControlTalon;
	Talon *intakeTalon;

	//double rightJoyY;
	double rightJoyX;
	double leftJoyY;
	//double leftJoyX;

public:
	Robot(void) {
		rightJoy = new Joystick(1);
		leftJoy = new Joystick(0);
		operatorJoy = new Joystick(2);
		intakeJoy = new Joystick(3);

		lfWheel = new Talon(8);
		lrWheel = new Talon(9);
		rfWheel = new Talon(0);
		rrWheel = new Talon(1);
		armControlTalon = new Talon(2);
		intakeTalon = new Talon(7);

		//rightJoyY = 0.0;
		rightJoyX = 0.0;
		leftJoyY = 0.0;
		//leftJoyX = 0.0;

	}

	void RobotInit() {

	}

	void AutonomousInit() {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {
		//leftJoyX = leftJoy->GetX();
		leftJoyY = leftJoy->GetY();
		rightJoyX = -rightJoy->GetX();
		//rightJoyY = rightJoy->GetY();

		lrWheel->SetSpeed(-leftJoyY - rightJoyX);
		lfWheel->SetSpeed(-leftJoyY - rightJoyX);
		rfWheel->SetSpeed(leftJoyY - rightJoyX);
		rrWheel->SetSpeed(leftJoyY - rightJoyX);

		//Lower buttons down
		if (operatorJoy->GetRawButton(3)) {
			armControlTalon->SetSpeed(0.4);
		}

		//Lower buttons up
		else if (operatorJoy->GetRawButton(4)) {
			armControlTalon->SetSpeed(-0.4);
		}

		else {
			armControlTalon->SetSpeed(0.0);
		}

		//Upper buttons down
		if (intakeJoy->GetRawButton(3)) {
			intakeTalon->SetSpeed(0.4);
		}

		//Upper buttons up
		else if (intakeJoy->GetRawButton(4)) {
			intakeTalon->SetSpeed(-0.4);
		}

		else {
			intakeTalon->SetSpeed(0.0);
		}
	}

	void TestPeriodic() {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
