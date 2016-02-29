#include "RobotModel.h"

#define PI 3.141592653589

RobotModel::RobotModel() {
	uint8_t update_rate_hz = 50;
	navx = new AHRS(SerialPort::Port::kMXP, AHRS::SerialDataType::kProcessedData, update_rate_hz);
	navx->ZeroYaw();

	leftEncoder = new Encoder(2, 3, true);
	rightEncoder = new Encoder(0, 1, true);
	leftEncoder->SetDistancePerPulse(((2.0/3.0) * PI) / 256.0);
	rightEncoder->SetDistancePerPulse(((2.0/3.0) * PI) / 256.0);

	leftA = new Talon(1);
	leftB = new Talon(0);
	rightA = new Talon(2);
	rightB = new Talon(5);

//	leftA = new Talon(7);
//	leftB = new Talon(8);
//	rightA = new Talon(2);
//	rightB = new Talon(1);

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

double RobotModel::GetLeftEncoderVal() {
	return leftEncoder->GetDistance();
}

double RobotModel::GetRightEncoderVal() {
	return rightEncoder->GetDistance();
}

void RobotModel::ResetDriveEncoders() {
	leftEncoder->Reset();
	rightEncoder->Reset();
}

RobotModel::~RobotModel() {
}
