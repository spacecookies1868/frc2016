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
//		autoPivotCommand = new AutoPivot(robot);
//		autoPivotCommand->SetDesiredYaw(30);
//		autoPivotCommand->Init();
	}

	void AutonomousPeriodic()
	{
		lastTimeSec = currTimeSec;
		currTimeSec = timer->Get();
		deltaTimeSec = currTimeSec - lastTimeSec;
		if (sensingBoulders->IsDone()) {
			robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		} else {
			sensingBoulders->Update(currTimeSec, lastTimeSec);
		}
//		if (autoPivotCommand->IsDone()){
//			robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
//		} else {
//			autoPivotCommand->Update(currTimeSec, deltaTimeSec);
//		}
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
