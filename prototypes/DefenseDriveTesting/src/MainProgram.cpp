#include <WPILib.h>
#include <AHRS.h>
#include "RobotModel.h"
#include "DefenseDrive.h"
#include "Logger.h"

class MainProgram: public IterativeRobot
{

public:
	RobotModel* robot;
	DefenseDrive* defenseDrive;
	double currTimeSec, lastTimeSec, deltaTimeSec;

	MainProgram() {
		robot = new RobotModel();
		defenseDrive = new DefenseDrive(robot);
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
	}

	void RobotInit()
	{

	}

	void AutonomousInit()
	{
		defenseDrive->Init();
	}

	void AutonomousPeriodic()
	{
		lastTimeSec = currTimeSec;
		currTimeSec = robot->GetTime();
		deltaTimeSec = currTimeSec - lastTimeSec;

		if (defenseDrive->IsDone()) {
			robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		} else {
			defenseDrive->Update(currTimeSec, deltaTimeSec);
			Logger::LogState(robot, defenseDrive);
		}
		SmartDashboard::PutNumber("Roll: %f\n", robot->GetRoll());
	}

	void TeleopInit()
	{
	}

	void TeleopPeriodic()
	{
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
	}

	void TestInit() {

	}

	void TestPeriodic()
	{
		SmartDashboard::PutNumber("Roll: %f\n", robot->GetRoll());
	}
};

START_ROBOT_CLASS(MainProgram)
