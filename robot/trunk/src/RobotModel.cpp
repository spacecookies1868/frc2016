#include "RobotModel.h"

#define PI 3.141592653589

#include "Debugging.h"

RobotModel::RobotModel() {
	pini = new Ini("/home/lvuser/robot.ini");

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

	frontLeftEncoder = new Encoder(LEFT_ENCODER_A_PWM_PORT,
				LEFT_ENCODER_B_PWM_PORT, true);
	frontRightEncoder = new Encoder(RIGHT_ENCODER_A_PWM_PORT,
				RIGHT_ENCODER_B_PWM_PORT, true);
	rearLeftEncoder = new Encoder(REAR_LEFT_ENCODER_A_PORT, REAR_LEFT_ENCODER_B_PORT, true);
	rearRightEncoder = new Encoder(REAR_RIGHT_ENCODER_A_PORT, REAR_RIGHT_ENCODER_B_PORT, true);

	frontLeftEncoder->SetDistancePerPulse(-(PI / 2.0) / 256.0);
	// 6 inch wheels (1/2 ft), 256 tics per rotation
	frontRightEncoder->SetDistancePerPulse((PI / 2.0) / 256.0);
	rearLeftEncoder->SetDistancePerPulse(-(PI / 2.0) / 256.0);
	rearRightEncoder->SetDistancePerPulse((PI / 2.0) / 256.0);
	compressor = new Compressor(COMPRESSOR_PORT);

	gyro = new Gyro(GYRO_PORT);
	gyro->Reset();
	gyro->SetSensitivity(0.007);

	timer = new Timer();
	timer->Start();
}

void RobotModel::SetWheelSpeed(Wheels w, double speed) {
/*
 * This is given that we will need to negate the speed for the two different sides.
 */

	switch (w) {
	case (kFrontLeftWheel):
#if COMP_BOT
		frontLeft->Set(speed);
#else
		frontLeft->Set(-speed);
#endif
		//SmartDashboard::PutNumber("frontLeft victor",
		//frontLeft->Get());
		break;
	case (kRearLeftWheel):
#if COMP_BOT
		rearLeft->Set(speed);
#else
		rearLeft->Set(-speed);
#endif
		//SmartDashboard::PutNumber("rearLeft victor", rearLeft->Get());
		break;
	case (kFrontRightWheel):
#if COMP_BOT
		frontRight->Set(-speed);
#else
		frontRight->Set(speed);
#endif
		//SmartDashboard::PutNumber("frontRight victor", frontRight->Get());
		break;
	case (kRearRightWheel):
#if COMP_BOT
		rearRight->Set(-speed);
#else
		rearRight->Set(speed);
#endif
		//SmartDashboard::PutNumber("rearRight victor", rearRight->Get());
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
	case (kFrontLeftWheel):
		return frontLeft->Get();
		break;
	case (kRearLeftWheel):
		return rearLeft->Get();
		break;
	case (kFrontRightWheel):
		return frontRight->Get();
		break;
	case (kRearRightWheel):
		return rearRight->Get();
		break;
	}
	return false;
}
double RobotModel::GetVoltage() {
	return pdp->GetVoltage();
}

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

void RobotModel::ResetGyro() {
	gyro->Reset();
}

void RobotModel::ResetTimer() {
	timer->Reset();
}

float RobotModel::GetGyroAngle() {
	return gyro->GetAngle();
}

double RobotModel::GetFrontLeftEncoderVal() {
	// from comp? return frontLeftEncoder->GetDistance();
	return frontLeftEncoder->GetDistance();
}

double RobotModel::GetFrontRightEncoderVal() {
	// from comp? return frontRightEncoder->GetDistance();
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

void RobotModel::RefreshIniFile() {
	delete pini;
	pini = new Ini("/home/lvuser/robot.ini");
}
