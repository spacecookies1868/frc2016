#include "WPILib.h"
//#define ARCADE_DRIVE

class Robot: public IterativeRobot
{

private:
	LiveWindow *lw = LiveWindow::GetInstance();

	Joystick *rightJoy;
	Joystick *leftJoy;
	Joystick *operatorJoy;
	Joystick *intakeJoy;

	Talon *leftDriveMotorA;
	Talon *leftDriveMotorB;
	Talon *rightDriveMotorA;
	Talon *rightDriveMotorB;
	Talon *armControlTalon;
	Talon *intakeTalon;

	double moveValue;
	double rotateValue;
	double leftMotorOutput;
	double rightMotorOutput;

	bool armButtonPressed;
	bool armSolenoidATrue;

	Compressor *compressor;

	Solenoid *armSolenoidA;
	Solenoid *armSolenoidB;


public:
	Robot(void) {
		rightJoy = new Joystick(1);
		leftJoy = new Joystick(0);
		operatorJoy = new Joystick(2);
		intakeJoy = new Joystick(3);

		leftDriveMotorA = new Talon(8);
		leftDriveMotorB = new Talon(9);
		rightDriveMotorA = new Talon(0);
		rightDriveMotorB = new Talon(1);
		armControlTalon = new Talon(7);
		intakeTalon = new Talon(2);

		moveValue = 0.0;
		rotateValue = 0.0;
		leftMotorOutput = 0.0;
		rightMotorOutput = 0.0;

		armButtonPressed = false;
		armSolenoidATrue = false;

		armSolenoidA = new Solenoid(0);
		armSolenoidB = new Solenoid(1);

		compressor = new Compressor(0);

	}

	void RobotInit() {
		compressor->Start();

	}

	void AutonomousInit() {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {

#ifdef ARCADE_DRIVE
		//Drive code: Arcade Drive
		moveValue =leftJoy->GetY();
		rotateValue = -rightJoy->GetX();

		leftMotorOutput = moveValue;
		rightMotorOutput = moveValue;

		leftMotorOutput += rotateValue;
		rightMotorOutput -= rotateValue;

		if (leftMotorOutput > 1.0) {
			rightMotorOutput = rightMotorOutput / leftMotorOutput;
			leftMotorOutput = 1.0;
		} else if (leftMotorOutput < -1.0) {
			rightMotorOutput = -rightMotorOutput / leftMotorOutput;
			leftMotorOutput = -1.0;
		} else if (rightMotorOutput > 1.0){
			leftMotorOutput = leftMotorOutput / rightMotorOutput;
			rightMotorOutput = 1.0;
		} else if (rightMotorOutput < -1.0) {
			leftMotorOutput = -leftMotorOutput/rightMotorOutput;
			rightMotorOutput = -1.0;
		}

		leftDriveMotorB->SetSpeed(-leftMotorOutput);
		leftDriveMotorA->SetSpeed(-leftMotorOutput);
		rightDriveMotorA->SetSpeed(rightMotorOutput);
		rightDriveMotorB->SetSpeed(rightMotorOutput);
#else
		leftMotorOutput = leftJoy->GetY();
		rightMotorOutput = rightJoy->GetY();

		leftDriveMotorB->SetSpeed(-leftMotorOutput);
		leftDriveMotorA->SetSpeed(-leftMotorOutput);
		rightDriveMotorA->SetSpeed(rightMotorOutput);
		rightDriveMotorB->SetSpeed(rightMotorOutput);


#endif
		//Motor control from operator and intake joysticks
		if (operatorJoy->GetRawButton(3)) {
			//Upper buttons down
			armControlTalon->SetSpeed(0.7);
		} else if (operatorJoy->GetRawButton(4)) {
			//Upper buttons up
			armControlTalon->SetSpeed(-0.4);
		} else {
			//Upper buttons in neutral position
			armControlTalon->SetSpeed(0.0);
		}

		if (intakeJoy->GetRawButton(3)) {
			//Lower buttons down
			intakeTalon->SetSpeed(-0.6);
		} else if (intakeJoy->GetRawButton(4)) {
			//Lower buttons up
			intakeTalon->SetSpeed(0.6);
		} else {
			//Lower buttons in neutral position
			intakeTalon->SetSpeed(0.0);
		}

		//Piston control
		if (operatorJoy->GetRawButton(8) && !armButtonPressed) {
			armSolenoidA->Set(!armSolenoidATrue);
			armSolenoidB->Set(armSolenoidATrue);
			armSolenoidATrue = !armSolenoidATrue;
			armButtonPressed = true;
		} else if (!operatorJoy->GetRawButton(8) && armButtonPressed) {
			armButtonPressed = false;
		}
	}

	void TestPeriodic() {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
