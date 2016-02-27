#ifndef SRC_ROBOTMODEL_H_
#define SRC_ROBOTMODEL_H_

#include <WPILib.h>
#include <AHRS.h>

class RobotModel {
public:
	enum Wheels {kLeftWheels, kRightWheels, kAllWheels};
	RobotModel();

	float GetYaw();
	float GetPitch();
	float GetRoll();

	void ZeroYaw();

	void SetWheelSpeed(Wheels w, double speed);
	float GetWheelSpeed(Wheels w);

	float GetTime();

	double GetLeftEncoderVal();
	double GetRightEncoderVal();
	void ResetDriveEncoders();

	virtual ~RobotModel();
private:
	AHRS *navx;
	Talon *leftA, *leftB, *rightA, *rightB;
	Timer *timer;
	Encoder *leftEncoder, *rightEncoder;
};

#endif
