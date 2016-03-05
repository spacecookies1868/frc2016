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
	bool GetServoDirection();
	void SetWheelSpeed(Wheels w, double speed);
	float GetYaw();
	float GetRoll();
	float GetPitch();
	void ZeroYaw();
//	float GetWheelSpeed(Wheels w);

private:
//	Victor *leftDriveMotorA, *leftDriveMotorB, *rightDriveMotorA, *rightDriveMotorB;
	Servo* servo;
	AHRS *navx;
	Talon *leftA, *leftB, *rightA, *rightB;

	double servoAngle;
	bool servoDirection;

};

#endif
