#include "WPILib.h"
#include "RobotModel.h"
#define ARCADE_DRIVE

class Robot: public IterativeRobot
{

private:
	LiveWindow *lw = LiveWindow::GetInstance();

	Joystick *rightJoy;
	Joystick *leftJoy;
	Joystick *operatorJoy;
	Joystick *intakeJoy;

	double moveValue;
	double rotateValue;
	double leftMotorOutput;
	double rightMotorOutput;

	bool armButtonPressed;
	bool armSolenoidATrue;

	RobotModel *myRobot;

public:
	Robot(void) {
		myRobot = new RobotModel();

		rightJoy = new Joystick(1);
		leftJoy = new Joystick(0);
		operatorJoy = new Joystick(2);
		intakeJoy = new Joystick(3);

		moveValue = 0.0;
		rotateValue = 0.0;
		leftMotorOutput = 0.0;
		rightMotorOutput = 0.0;

		armButtonPressed = false;
		armSolenoidATrue = false;


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

		myRobot->SetWheelSpeed(RobotModel::kLeftWheels, leftMotorOutput);
		myRobot->SetWheelSpeed(RobotModel::kRightWheels, rightMotorOutput);
#else //Tank drive
		leftMotorOutput = leftJoy->GetY();
		rightMotorOutput = rightJoy->GetY();

		myRobot->SetWheelSpeed(RobotModel::kLeftWheels, leftMotorOutput);
		myRobot->SetWheelSpeed(RobotModel::kRightWheels, rightMotorOutput);

#endif
		//Motor control from operator and intake joysticks
		if (operatorJoy->GetRawButton(3)) {
			//Upper buttons down
			myRobot->SetArmControlSpeed(0.7);
		} else if (operatorJoy->GetRawButton(4)) {
			//Upper buttons up
			myRobot->SetArmControlSpeed(-0.4);
		} else {
			//Upper buttons in neutral position
			myRobot->SetArmControlSpeed(0.0);
		}

		if (intakeJoy->GetRawButton(3)) {
			//Lower buttons down
			myRobot->SetIntakeSpeed(-0.6);
		} else if (intakeJoy->GetRawButton(4)) {
			//Lower buttons up
			myRobot->SetIntakeSpeed(0.6);
		} else {
			//Lower buttons in neutral position
			myRobot->SetIntakeSpeed(0.0);
		}

		//Piston control
		if (operatorJoy->GetRawButton(8) && !armButtonPressed) {
			myRobot->SetArmSolenoid(!armSolenoidATrue);
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
