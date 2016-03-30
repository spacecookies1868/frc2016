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
	void SetAutoManualOuttakeForward(bool desired);
	void SetAutoManualOuttakeReverse(bool desired);

	//auto boolean accessor methods
	bool GetOuttakeFinished();

	enum SuperstructureState {
		kInit, kIdle, kAutoPrepToOuttake, kTeleopPrepToOuttake, kEncOuttake, kAutoTimeOuttake,
		kTeleopTimeOuttake
	};

private:
	RobotModel* robot;
	RemoteControl* humanControl;

	uint32_t m_stateVal;
	uint32_t nextState;

	bool useDoorbellButtons;

	//PTO = prep to outtake
	bool startedPTO;
	double startTimePTO;
	double deltaTimePTO;

	//OT = outtake
	bool startedOT;
	double startEncoderValEOT;
	double deltaEncoderValEOT;
	double startTimeTOT;
	double deltaTimeTOT;

	//auto booleans
	bool autoDefenseManipUp;
	bool autoDefenseManipDown;
	bool autoIntakeUp;
	bool autoIntakeDown;
	bool autoIntakeMotorForward;
	bool autoIntakeMotorBackward;
	bool autoOuttake;
	bool autoOuttakeFinished;
	bool autoManualOuttakeForward;
	bool autoManualOuttakeReverse;

	//motor speeds
	double intakeSpeed;
	double outtakeSpeed;

};

#endif
