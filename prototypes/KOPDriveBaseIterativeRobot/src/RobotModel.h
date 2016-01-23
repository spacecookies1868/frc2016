#include "WPILib.h"

#ifndef ROBOTMODEL_H_
#define ROBOTMODEL_H_

class RobotModel {
public:
	enum Wheels {kLeftWheels, kRightWheels, kAllWheels};

	RobotModel();
	~RobotModel() {}

	double GetTime();

	void SetWheelSpeed(Wheels w, double speed);
	float GetWheelSpeed(Wheels w);

	void SetArmControlSpeed(double speed);
	float GetArmControlSpeed();

	void SetIntakeSpeed(double speed);
	float GetIntakeSpeed();

	void SetArmSolenoid(bool desired);
	bool GetArmSolenoid();

private:
	Talon *leftDriveMotorA;
	Talon *leftDriveMotorB;
	Talon *rightDriveMotorA;
	Talon *rightDriveMotorB;
	Talon *armControlTalon;
	Talon *intakeTalon;

	Compressor *compressor;

	Solenoid *armSolenoidA;
	Solenoid *armSolenoidB;

	Timer *timer;

};



#endif /* ROBOTMODEL_H_ */
