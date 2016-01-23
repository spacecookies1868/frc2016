#include "RobotModel.h"

RobotModel::RobotModel() {
	leftDriveMotorA = new Talon(8);
	leftDriveMotorB = new Talon(9);
	rightDriveMotorA = new Talon(0);
	rightDriveMotorB = new Talon(1);
	armControlTalon = new Talon(7);
	intakeTalon = new Talon(2);

	armSolenoidA = new Solenoid(0);
	armSolenoidB = new Solenoid(1);

	compressor = new Compressor(0);

	timer = new Timer();

}

void RobotModel::SetWheelSpeed(Wheels w, double speed) {
	switch(w) {
	case(kLeftWheels):
		leftDriveMotorA->Set(-speed);
		leftDriveMotorB->Set(-speed);
		break;
	case(kRightWheels):
		rightDriveMotorA->Set(speed);
		rightDriveMotorB->Set(speed);
		break;
	case(kAllWheels):
		leftDriveMotorA->Set(-speed);
		leftDriveMotorB->Set(-speed);
		rightDriveMotorA->Set(speed);
		rightDriveMotorB->Set(speed);
		break;
	}
}

float RobotModel::GetWheelSpeed(Wheels w) {
	switch(w) {
	case(kLeftWheels):
		return leftDriveMotorA->Get();
	case(kRightWheels):
		return rightDriveMotorA->Get();
	default:
		return 0.0;
	}
}

double RobotModel::GetTime() {
	return timer->Get();
}

void RobotModel::SetArmControlSpeed(double speed) {
	armControlTalon->Set(speed);
}

float RobotModel::GetArmControlSpeed() {
	return armControlTalon->Get();
}

void RobotModel::SetIntakeSpeed(double speed) {
	intakeTalon->Set(speed);
}

float RobotModel::GetIntakeSpeed() {
	return intakeTalon->Get();
}

void RobotModel::SetArmSolenoid(bool desired) {
	armSolenoidA->Set(desired);
	armSolenoidB->Set(!desired);
}

bool RobotModel::GetArmSolenoid() {
	return armSolenoidA->Get();
}

