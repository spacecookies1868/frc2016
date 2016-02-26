#include "RobotModel.h"

RobotModel::RobotModel() {
	uint8_t update_rate_hz = 50;
	navx = new AHRS(SerialPort::Port::kMXP, AHRS::SerialDataType::kProcessedData, update_rate_hz);
	navx->ZeroYaw();

	leftA = new Talon(1);
	leftB = new Talon(0);
	rightA = new Talon(2);
	rightB = new Talon(5);
	timer = new Timer();
	timer->Start();
}

void RobotModel::SetWheelSpeed(Wheels w, double speed) {
	switch (w) {
	case (kLeftWheels):
		leftA->Set(speed);
		leftB->Set(speed);
		break;
	case (kRightWheels):
		rightA->Set(-speed);
		rightB->Set(-speed);
		break;
	case (kAllWheels):
		leftA->Set(speed);
		leftB->Set(speed);
		rightA->Set(-speed);
		rightB->Set(-speed);
		break;
	}
}

float RobotModel::GetWheelSpeed(Wheels w) {
	//IMPORTANT: Check which motors need to be inverted.
	switch(w) {
	case (kLeftWheels):
		return leftA->Get();
		break;
	case (kRightWheels):
		return rightA->Get();
		break;
	default:
		return 0.0;
		break;
	}
}

float RobotModel::GetYaw() {
	return navx->GetYaw();
}

float RobotModel::GetPitch() {
	return navx->GetPitch();
}

float RobotModel::GetRoll() {
	return navx->GetRoll();
}

void RobotModel::ZeroYaw() {
	navx->ZeroYaw();
}

float RobotModel::GetTime() {
	return timer->Get();
}

RobotModel::~RobotModel() {
}
