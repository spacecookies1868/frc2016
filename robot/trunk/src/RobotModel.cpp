#include "RobotModel.h"

#define PI 3.141592653589

#include "Debugging.h"

RobotModel::RobotModel() {
	pdp = new PowerDistributionPanel();

	frontLeft = new Victor(FRONT_LEFT_MOTOR_PWM_PORT);
	rearLeft = new Victor(REAR_LEFT_MOTOR_PWM_PORT);
	frontRight = new Victor(FRONT_RIGHT_MOTOR_PWM_PORT);
	rearRight = new Victor(REAR_RIGHT_MOTOR_PWM_PORT);

	frontLeft->SetExpiration(0.5);
	rearLeft->SetExpiration(0.5);
	frontRight->SetExpiration(0.5);
	rearRight->SetExpiration(0.5);

	frontLeft->SetSafetyEnabled(false);
	rearLeft->SetSafetyEnabled(false);
	frontRight->SetSafetyEnabled(false);
	rearRight->SetSafetyEnabled(false);

	isLowGear = false;
	shiftGear = new Solenoid(GEAR_SHIFTER_SOLENOID_PORT);

	/**
	frontLeftEncoder = new Encoder(LEFT_ENCODER_A_PWM_PORT, LEFT_ENCODER_B_PWM_PORT, true);
	frontRightEncoder = new Encoder(RIGHT_ENCODER_A_PWM_PORT, RIGHT_ENCODER_B_PWM_PORT, true);
	rearLeftEncoder = new Encoder(REAR_LEFT_ENCODER_A_PORT, REAR_LEFT_ENCODER_B_PORT, true);
	rearRightEncoder = new Encoder(REAR_RIGHT_ENCODER_A_PORT, REAR_RIGHT_ENCODER_B_PORT, true);

	// 6 inch wheels (1/2 ft), 256 tics per rotation
	frontLeftEncoder->SetDistancePerPulse(-(PI / 2.0) / 256.0);
	frontRightEncoder->SetDistancePerPulse((PI / 2.0) / 256.0);
	rearLeftEncoder->SetDistancePerPulse(-(PI / 2.0) / 256.0);
	rearRightEncoder->SetDistancePerPulse((PI / 2.0) / 256.0);
	**/
//	compressor = new Compressor(COMPRESSOR_PORT);
	serialPort = new SerialPort(57600, SerialPort::kMXP);

	timer = new Timer();
	timer->Start();
}

void RobotModel::SetWheelSpeed(Wheels w, double speed) {
	switch (w) {
	case (kLeftWheels):
		frontLeft->Set(speed);
		rearLeft->Set(speed);
		break;
	case (kRightWheels):
		frontRight->Set(speed);
		rearRight->Set(speed);
		break;
	case (kAllWheels):
		frontLeft->Set(speed);
		rearLeft->Set(speed);
		frontRight->Set(speed);
		rearRight->Set(speed);
		break;
	}
}

float RobotModel::GetWheelSpeed(Wheels w) {
	switch(w) {
	case (kLeftWheels):
		return frontLeft->Get();
		break;
	case (kRightWheels):
		return frontRight->Get();
		break;
	}
	return false;
}

bool RobotModel::IsLowGear() {
	return isLowGear;
}

void RobotModel::ShiftToHighGear() {
	shiftGear->Set(true);
}

void RobotModel::ShiftToLowGear() {
	shiftGear->Set(false);
}

double RobotModel::GetVoltage() {
	return pdp->GetVoltage();
}
/*
void RobotModel::EnableCompressor() {
	compressor->Start();
}

void RobotModel::DisableCompressor() {
	compressor->Stop();
}

void RobotModel::ResetCompressor() {
	compressor->Stop();
	compressor->Start();
}

bool RobotModel::GetCompressorState() {
	return (compressor->Enabled());
}
*/
void RobotModel::ResetTimer() {
	timer->Reset();
}

/**
double RobotModel::GetFrontLeftEncoderVal() {
	return frontLeftEncoder->GetDistance();
}

double RobotModel::GetFrontRightEncoderVal() {
	return frontRightEncoder->GetDistance();
}

double RobotModel::GetRearLeftEncoderVal() {
	return -rearLeftEncoder->GetDistance();
}

double RobotModel::GetRearRightEncoderVal() {
	return -rearRightEncoder->GetDistance();
}

void RobotModel::ResetDriveEncoders() {
	frontLeftEncoder->Reset();
	frontRightEncoder->Reset();
	rearLeftEncoder->Reset();
	rearRightEncoder->Reset();
}
*/

void RobotModel::RefreshIni() {
//	delete pini;
//	pini = new Ini("/home/lvuser/robot.ini");
}
