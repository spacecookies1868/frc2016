#include "WPILib.h"
#include "RobotModel.h"
#include "SensingBoulders.h"
#include "UltrasonicSensor.h"
#include "AutoPivot.h"

class MainProgram: public IterativeRobot
{
private:
	RobotModel* robot;
	SensingBoulders* sensingBoulders;
	AutoPivot* autoPivotCommand;
	Timer* timer;
	double currTimeSec, lastTimeSec, deltaTimeSec;

	bool isPivotInitialized;

public:
	void RobotInit()
	{
		printf("Initializing Robot \n");
		robot = new RobotModel();
		printf ("About to create sensingBoulders \n");
		sensingBoulders = new SensingBoulders(robot);
		printf("Finished creating sensingBoulders \n");

		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
		timer->Start();
		printf("Set times & start timer");
	}

	void DisabledInit()
	{
//		printf("Calling disabledInit \n");
	}

	void AutonomousInit()
	{
		sensingBoulders->Init();

		isPivotInitialized = false;
	}

	void AutonomousPeriodic()
	{
		if (sensingBoulders->IsDone()) {
//			printf("center boulder angle: %f\n", sensingBoulders->GetCenterBoulderAngle());
//			printf("center boulder distance: %f\n", sensingBoulders->GetCenterBoulderDistance());
			if (isPivotInitialized){
				lastTimeSec = currTimeSec;
				currTimeSec = timer->Get();
				deltaTimeSec = currTimeSec - lastTimeSec;
				if (autoPivotCommand->IsDone(deltaTimeSec)) {
					robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
				} else {
					autoPivotCommand->Update(currTimeSec, deltaTimeSec);
				}
			} else {
				robot->ZeroYaw();
 				double myDesiredAngle = sensingBoulders->GetCenterBoulderAngle() - 90;
				autoPivotCommand = new AutoPivot(robot, myDesiredAngle);
				autoPivotCommand->Init();

				isPivotInitialized = true;
			}
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
