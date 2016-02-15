#ifndef SUPERSTRUCTURECONTROLLER_H_
#define SUPERSTRUCTURECONTROLLER_H_

#include "RobotModel.h"
#include "RemoteControl.h"
#include "Debugging.h"

class SuperstructureController {
public:
	SuperstructureController(RobotModel* myRobot, RemoteControl* myHumanControl);
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
	void SetAutoOuttakeIn(bool desired);
	void SetAutoOuttakeOut(bool desired);

	enum SuperstructureState {
		kInit, kIdle, kPrepAndOuttake
	};

private:
	RobotModel* robot;
	RemoteControl* humanControl;

	uint32_t m_stateVal;
	uint32_t nextState;

	//PAO = prep and outtake
	bool startedPAO;
	double startTimePAO;
	double deltaTimePAO;

	//auto booleans
	bool autoDefenseManipUp;
	bool autoDefenseManipDown;
	bool autoIntakeUp;
	bool autoIntakeDown;
	bool autoIntakeMotorForward;
	bool autoIntakeMotorBackward;
	bool autoOuttakeIn;
	bool autoOuttakeOut;

};

#endif
