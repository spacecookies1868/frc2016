#ifndef SRC_ROBOTMODEL_H_
#define SRC_ROBOTMODEL_H_

#include "WPILib.h"
#include <AHRS.h>

class RobotModel {
public:
	enum Wheels {kLeftWheels, kRightWheels, kAllWheels};

	RobotModel();
	virtual ~RobotModel();

	void SetWheelSpeed(Wheels w, double speed);
	float GetYaw();
	float GetRoll();
	float GetPitch();
	void ZeroYaw();

private:
	AHRS *navx;
	Talon *leftA, *leftB, *rightA, *rightB;
};

#endif
