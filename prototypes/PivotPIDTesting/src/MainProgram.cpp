#include "WPILib.h"
#include "RobotModel.h"
#include "AutoPivot.h"
#include <AHRS.h>

class MainProgram: public IterativeRobot
{
	RobotModel* robot;
	AutoPivot* autoPivotCommand;
	Timer* timer;
	double currTimeSec, lastTimeSec, deltaTimeSec;

public:
	MainProgram() {
		robot = new RobotModel();
		timer = new Timer();
	}

	void RobotInit()
	{
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
		timer->Start();
	}

	void AutonomousInit()
	{
		robot->ZeroYaw();
		double myDesiredAngle = -90;
		autoPivotCommand = new AutoPivot(robot, myDesiredAngle);
		autoPivotCommand->Init();
		printf("Initial yaw: %f\n", robot->GetYaw());
	}

	void AutonomousPeriodic()
	{
		lastTimeSec = currTimeSec;
		currTimeSec = timer->Get();
		deltaTimeSec = currTimeSec - lastTimeSec;
		if (autoPivotCommand->IsDone(deltaTimeSec)) {
			robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		} else {
			autoPivotCommand->Update(currTimeSec, deltaTimeSec);
		}
	}

	void TeleopInit() {
	}

	void TeleopPeriodic() {
	}

	void TestPeriodic()
	{
	}

	void DisabledPeriodic() {
	}
};

START_ROBOT_CLASS(MainProgram)
