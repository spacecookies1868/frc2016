#include "WPILib.h"
#include "RobotModel.h"
#include "SensingBoulders.h"
#include "UltrasonicSensor.h"

class MainProgram: public IterativeRobot
{
private:
	RobotModel* robot;
	SensingBoulders* sensingBoulders;

	void RobotInit()
	{
		robot = new RobotModel();
		sensingBoulders = new SensingBoulders(robot);
	}

	void AutonomousInit()
	{
		sensingBoulders->Init();
	}

	void AutonomousPeriodic()
	{
		if (sensingBoulders->IsDone()) {
//			printf("center boulder angle: %f\n", sensingBoulders->GetCenterBoulderAngle());
//			printf("center boulder distance: %f\n", sensingBoulders->GetCenterBoulderDistance());
		} else {
			sensingBoulders->Update();
//			printf("update\n");
		}
	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
	}

	void DisableTeleop() {

	}

	void TestPeriodic()
	{
	}
};

START_ROBOT_CLASS(MainProgram)
