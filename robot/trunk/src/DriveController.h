#ifndef DRIVECONTROLLER_H_
#define DRIVECONTROLLER_H_

#include "RobotModel.h"
#include "RemoteControl.h"
#include "Debugging.h"
#include "PIDControlLoop.h"

class DriveController {
public:
	DriveController(RobotModel*, RemoteControl*);
	void Update(double currTimeSec, double deltaTimeSec);
	void RefreshIni();
	void Reset();
	void QuickTurn(double myRight);
	void ArcadeDrive(double myX, double myY);
	void TankDrive(double myLeft, double myRight);

	int DriveDirection();
	virtual ~DriveController();

	enum DriveState {
		kInitialize, kTeleopDrive
	};

private:
	RobotModel *robot;
	RemoteControl *humanControl;

	PIDConfig* CreateRPIDConfig();
	PIDConfig* rPIDConfig;
	PIDControlLoop* rPID;
	double rPFac, rIFac, rDFac, rDesiredAccuracy, rMaxAbsOutput, rMaxAbsDiffError,
			rMaxAbsError, rMaxAbsITerm, rTimeLimit;
	double lastR;
	bool initializedRPID;

	uint32_t m_stateVal;
	uint32_t nextState;
};

#endif
