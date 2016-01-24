#include "WPILib.h"
#include "RobotModel.h"
#include "ControlBoard.h"
#define ARCADE_DRIVE

class Robot: public IterativeRobot
{

private:
	LiveWindow *lw = LiveWindow::GetInstance();

	double moveValue;
	double rotateValue;
	double leftMotorOutput;
	double rightMotorOutput;

	bool armSolenoidATrue;

	RobotModel *myRobot;
	RemoteControl *humanControl;

public:
	Robot(void) {
		myRobot = new RobotModel();
		humanControl = new ControlBoard();

		moveValue = 0.0;
		rotateValue = 0.0;
		leftMotorOutput = 0.0;
		rightMotorOutput = 0.0;

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
		moveValue = humanControl->GetJoystickValues(RemoteControl::kLeftJoy, RemoteControl::kY);
		rotateValue = humanControl->GetJoystickValues(RemoteControl::kRightJoy, RemoteControl::kX);

		leftMotorOutput = moveValue;
		rightMotorOutput = moveValue;

		leftMotorOutput -= rotateValue;
		rightMotorOutput += rotateValue;

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
		leftMotorOutput = humanControl->GetJoystickValues(RemoteControl::kLeftJoy, RemoteControl::kY);
		rightMotorOutput = humanControl->GetJoystickValues(RemoteControl::kLeftJoy, RemoteControl::kY);

		myRobot->SetWheelSpeed(RobotModel::kLeftWheels, leftMotorOutput);
		myRobot->SetWheelSpeed(RobotModel::kRightWheels, rightMotorOutput);

#endif
		//Motor control from operator and intake joysticks
		if (humanControl->GetArmControlButtonDown()) {
			//Upper buttons down
			myRobot->SetArmControlSpeed(0.7);
		} else if (humanControl->GetArmControlButtonUp()) {
			//Upper buttons up
			myRobot->SetArmControlSpeed(-0.4);
		} else {
			//Upper buttons in neutral position
			myRobot->SetArmControlSpeed(0.0);
		}

		if (humanControl->GetIntakeButtonIn()) {
			//Lower buttons down
			myRobot->SetIntakeSpeed(-0.6);
		} else if (humanControl->GetIntakeButtonOut()) {
			//Lower buttons up
			myRobot->SetIntakeSpeed(0.6);
		} else {
			//Lower buttons in neutral position
			myRobot->SetIntakeSpeed(0.0);
		}

		//Piston control
		if (humanControl->GetArmControlButtonPressed()) {
			myRobot->SetArmSolenoid(!armSolenoidATrue);
			armSolenoidATrue = !armSolenoidATrue;
		}
	}

	void TestPeriodic() {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
