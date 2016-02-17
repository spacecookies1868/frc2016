#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include "WPILib.h"
#include <AHRS.h>

class RobotModel {
public:
	enum Wheels {kLeftWheels, kRightWheels, kAllWheels};

	RobotModel();
	~RobotModel() {}

	//void Reset();
	void InitServo(double angle);
	void SetServo(double startAngle, double endAngle, double deltaAngle);
	double GetServoAngle();
	void SetWheelSpeed(Wheels w, double speed);
	float GetYaw();
	float GetRoll();
	float GetPitch();
	void ZeroYaw();
//	float GetWheelSpeed(Wheels w);

private:
//	Victor *leftDriveMotorA, *leftDriveMotorB, *rightDriveMotorA, *rightDriveMotorB;
	Servo* servo;
	double servoAngle;
	bool servoDirection;
	AHRS *navx;
	Talon *leftA, *leftB, *rightA, *rightB;

};

#endif
