#include "RobotModel.h"


RobotModel::RobotModel() {
	servo = new Servo(6);
	servoAngle = servo->GetAngle();
	servoDirection = true;

	uint8_t update_rate_hz = 50;
	navx = new AHRS(SerialPort::Port::kMXP, AHRS::SerialDataType::kProcessedData, update_rate_hz);
	navx->ZeroYaw();

	leftA = new Talon(1);
	leftB = new Talon(0);
	rightA = new Talon(2);
	rightB = new Talon(5);

}

void RobotModel::InitServo(double angle) {
	servo->SetAngle(angle);
}

void RobotModel::SetServo(double startAngle, double endAngle, double deltaAngle) {
	servoAngle = servo->GetAngle();
	if (servoAngle > endAngle){
		servoDirection = false;
	} else if (servoAngle < startAngle){
		servoDirection = true;
	}

	if (servoDirection == true){
		servoAngle += deltaAngle;
	} else {
		servoAngle -= deltaAngle;
	}
	servo->SetAngle(servoAngle);
}

double RobotModel::GetServoAngle() {
	servoAngle = servo->GetAngle();
	return servoAngle;
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

