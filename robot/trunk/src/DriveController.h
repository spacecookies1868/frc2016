#ifndef DRIVECONTROLLER_H_
#define DRIVECONTROLLER_H_

#include "RobotModel.h"
#include "RemoteControl.h"
#include "Debugging.h"

class DriveController{
public:
	DriveController(RobotModel*, RemoteController*);
	void Update(double currTimeSec, double deltaTimeSec);
	void RefreshIni();
	void Reset();

	virtual ~DriveController();

	enum DriveState {
		kReset, kInitialize, kTeleopDrive
	};

private:
	RobotModel *robot;
	RemoteController *humanControl;

	uint32_t m_stateVal;
	uint32_t nextState;
};

#endif
