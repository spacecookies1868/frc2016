#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include "WPILib.h"

class RobotModel {
public:
	//enum Wheels {kLeftWheels, kRightWheels, kAllWheels};

	RobotModel();
	~RobotModel() {}

//	void SetWheelSpeed(Wheels w, double speed);
//	float GetWheelSpeed(Wheels w);

	//void Reset();
	void SetServo(double startAngle, double endAngle, double deltaAngle);
	double GetServoAngle();

private:
//	Victor *leftDriveMotorA, *leftDriveMotorB, *rightDriveMotorA, *rightDriveMotorB;
	Servo* servo;
	double servoAngle;
	bool servoDirection;
};

#endif
