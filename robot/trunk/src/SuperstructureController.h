#ifndef SUPERSTRUCTURECONTROLLER_H_
#define SUPERSTRUCTURECONTROLLER_H_

#include "RobotModel.h"
#include "RemoteControl.h"
#include "Debugging.h"

class SuperstructureController {
public:
	SuperstructureController(RobotModel* myRobot, RemoteControl* myHumanControl);
	~SuperstructureController() {};
	void Reset();
	void Update(double currTimeSec, double deltaTimeSec);
	void RefreshIni();

	//auto boolean mutator methods
	void SetAutoDefenseManipUp(bool desired);
	void SetAutoDefenseManipDown(bool desired);
	void SetAutoIntakeUp(bool desired);
	void SetAutoIntakeDown(bool desired);
	void SetAutoIntakeMotorForward(bool desired);
	void SetAutoIntakeMotorBackward(bool desired);
	void SetAutoOuttake(bool desired);

	//auto boolean accessor methods
	bool GetOuttakeFinished();

	enum SuperstructureState {
		kInit, kIdle, kPrepToOuttake, kOuttake
	};

private:
	RobotModel* robot;
	RemoteControl* humanControl;

	uint32_t m_stateVal;
	uint32_t nextState;

	//PTO = prep to outtake
	bool startedPTO;
	double startTimePTO;
	double deltaTimePTO;

	//OT = outtake
	bool startedOT;
	double startEncoderValOT;
	double deltaEncoderValOT;

	//auto booleans
	bool autoDefenseManipUp;
	bool autoDefenseManipDown;
	bool autoIntakeUp;
	bool autoIntakeDown;
	bool autoIntakeMotorForward;
	bool autoIntakeMotorBackward;
	bool autoOuttake;
	bool autoOuttakeFinished;

	//motor speeds
	double intakeSpeed;
	double outtakeSpeed;

};

#endif
