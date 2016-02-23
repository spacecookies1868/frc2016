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
	MainProgram()
	{
//		printf("Initializing Robot\n");
		robot = new RobotModel();
//		printf("Finished creating robot\n");
	}
	void RobotInit()
	{
//		printf ("About to create sensingBoulders \n");
		sensingBoulders = new SensingBoulders(robot);
//		printf("Finished creating sensingBoulders \n");
		timer = new Timer();
//		printf("Finished creating timer\n");

		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
		timer->Start();
		printf("Set times & start timer");

		robot->ZeroYaw();
	}

	void AutonomousInit()
	{
		sensingBoulders->Init();
	}

	void AutonomousPeriodic()
	{
		currTimeSec = timer->Get();
		if (sensingBoulders->IsDone()) {
			return;
		} else {
			sensingBoulders->Update(currTimeSec, lastTimeSec);
		}
		lastTimeSec = currTimeSec;
	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		printf("Yaw Value %f\n", robot->GetYaw());
	}
	void DisabledInit() {
	}
	void DisabledPeriodic() {
	}

	void TestPeriodic()
	{
		SmartDashboard::PutNumber("Sonic Reading", sensingBoulders->GetDistanceInches());
	}
};

START_ROBOT_CLASS(MainProgram)
