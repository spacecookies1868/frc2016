#include "RobotModel.h"


RobotModel::RobotModel() {
	servo = new Servo(6);
	servoAngle = servo->GetAngle();
	servoDirection = true;
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
	return servoAngle;
}
