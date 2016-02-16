#include "WPILib.h"
#include "RobotModel.h"
#include "SensingBoulders.h"
#include "UltrasonicSensor.h"

class MainProgram: public IterativeRobot
{
private:
	RobotModel* robot;
	SensingBoulders* sensingBoulders;

public:
	void RobotInit()
	{
//		printf("Initializing Robot \n");
		robot = new RobotModel();
//		printf ("About to create sensingBoulders \n");
		sensingBoulders = new SensingBoulders(robot);
//		printf("Finished creating sensingBoulders \n");
	}

	void DisabledInit()
	{
//		printf("Calling disabledInit \n");
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
		SmartDashboard::PutNumber("Sonic Reading", sensingBoulders->GetDistanceInches());
	}
};

START_ROBOT_CLASS(MainProgram)
