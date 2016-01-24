#include "RobotModel.h"

#define PI 3.141592653589

#include "Debugging.h"

RobotModel::RobotModel() {
	pdp = new PowerDistributionPanel();

	leftDriveMotorA = new Victor(LEFT_DRIVE_MOTOR_A_PWM_PORT);
	leftDriveMotorB = new Victor(LEFT_DRIVE_MOTOR_B_PWM_PORT);
	rightDriveMotorA = new Victor(RIGHT_DRIVE_MOTOR_A_PWM_PORT);
	rightDriveMotorB = new Victor(RIGHT_DRIVE_MOTOR_B_PWM_PORT);

	leftDriveMotorA->SetExpiration(0.5);
	leftDriveMotorB->SetExpiration(0.5);
	rightDriveMotorA->SetExpiration(0.5);
	rightDriveMotorB->SetExpiration(0.5);

	leftDriveMotorA->SetSafetyEnabled(false);
	leftDriveMotorB->SetSafetyEnabled(false);
	rightDriveMotorA->SetSafetyEnabled(false);
	rightDriveMotorB->SetSafetyEnabled(false);

	isLowGear = false;
	gearShiftSolenoid = new Solenoid(GEAR_SHIFT_SOLENOID_PORT);

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

void RobotModel::Reset() {
	isLowGear = !gearShiftSolenoid->Get();
}

void RobotModel::SetWheelSpeed(Wheels w, double speed) {
	//IMPORTANT: Check which motors need to be inverted.
	switch (w) {
	case (kLeftWheels):
		leftDriveMotorA->Set(speed);
		leftDriveMotorB->Set(speed);
		break;
	case (kRightWheels):
		rightDriveMotorA->Set(speed);
		rightDriveMotorB->Set(speed);
		break;
	case (kAllWheels):
		leftDriveMotorA->Set(speed);
		leftDriveMotorB->Set(speed);
		rightDriveMotorA->Set(speed);
		rightDriveMotorB->Set(speed);
		break;
	}
}

float RobotModel::GetWheelSpeed(Wheels w) {
	//IMPORTANT: Check which motors need to be inverted.
	switch(w) {
	case (kLeftWheels):
		return leftDriveMotorA->Get();
		break;
	case (kRightWheels):
		return rightDriveMotorA->Get();
		break;
	default:
		return 0.0;
		break;
	}
	return false;
}

bool RobotModel::IsLowGear() {
	return isLowGear;
}

void RobotModel::ShiftToHighGear() {
	gearShiftSolenoid->Set(true);
	isLowGear = false;
}

void RobotModel::ShiftToLowGear() {
	gearShiftSolenoid->Set(false);
	isLowGear = true;
}

double RobotModel::GetVoltage() {
	return pdp->GetVoltage();
}

void RobotModel::ResetTimer() {
	timer->Reset();
}

double RobotModel::GetTime() {
	return timer->Get();
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
	/*delete pini;
	pini = new Ini("/home/lvuser/robot.ini");*/
}
