#include "RobotModel.h"


RobotModel::RobotModel() {
	servo = new Servo(6);

	servoAngle = servo->GetAngle();
	servoDirection = true;

//	printf("Creating navx\n");
	navx = new AHRS(SPI::Port::kMXP);
//	printf("Finished creating navx\n");
	navx->ZeroYaw();
//	printf("Zeroed the yaw\n");

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
	if (servoAngle >= endAngle){
		servoDirection = false;
	} else if (servoAngle <= startAngle){
		servoDirection = true;
	}

	if (servoDirection == true){
		servoAngle += deltaAngle;
	} else {
		servoAngle -= deltaAngle;
	}
//	printf("Start angle: %f \n", startAngle);
//	printf("End Angle: %f \n", endAngle);
//	printf("Delta Angle: %f \n", deltaAngle);
//	printf("servoAnlge: %f \n", servoAngle);
	servo->SetAngle(servoAngle);
}

double RobotModel::GetServoAngle() {
	servoAngle = servo->GetAngle();
	return servoAngle;
}

bool RobotModel::GetServoDirection() {
	return servoDirection;
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

