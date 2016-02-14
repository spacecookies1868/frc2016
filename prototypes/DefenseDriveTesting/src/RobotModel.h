#ifndef SRC_ROBOTMODEL_H_
#define SRC_ROBOTMODEL_H_

#include <WPILib.h>
#include <AHRS.h>

class RobotModel {
public:
	enum Wheels {kLeftWheels, kRightWheels, kAllWheels};
	RobotModel();
	float GetYaw();
	float GetRoll();
	float GetPitch();
	void ZeroYaw();
	void SetWheelSpeed(Wheels w, double speed);
	virtual ~RobotModel();
private:
	AHRS *navx;
	Talon *leftA, *leftB, *rightA, *rightB;
};

#endif
