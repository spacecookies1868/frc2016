#include "WPILib.h"
#include <ControlBoard.h>
#include <RobotModel.h>
#include <AutoPivot.h>

class MainProgram: public IterativeRobot
{
private:
	ControlBoard* controlBoard;
	AutoPivot* autoPivotCommand;
	RobotModel* robot;
	Timer* timer;
	double currTimeSec, lastTimeSec, deltaTimeSec;
	bool isPivoting;

	void RobotInit()
	{
		controlBoard = new ControlBoard();
		robot = new RobotModel();
		autoPivotCommand = new AutoPivot(robot);
		timer = new Timer();
		timer->Start();
		isPivoting = false;
	}


	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{
		robot->ZeroYaw();
		currTimeSec = timer->Get();
		deltaTimeSec = 0.0;
		lastTimeSec = currTimeSec;
		isPivoting = false;
	}

	void TeleopPeriodic()
	{
		lastTimeSec = currTimeSec;
		currTimeSec = timer->Get();
		deltaTimeSec = currTimeSec - lastTimeSec;
		controlBoard->ReadControls();
		SmartDashboard::PutNumber("Dial Value", controlBoard->GetDesiredAngle());
		SmartDashboard::PutNumber("Current Yaw", robot->GetYaw());
		if (controlBoard->GetPivotDesired()) {
			printf("Desired Angle: %f\n", controlBoard->GetDesiredAngle());
			printf("Current Angle: %f\n", robot->GetYaw());
			autoPivotCommand->SetDesiredYaw(controlBoard->GetDesiredAngle());
			printf("Desired Yaw Set \n");
			autoPivotCommand->Init();
			printf("AutoPivot Initialized \n");
			isPivoting = true;
			printf("isPivoting set to true \n");
		}
		if (isPivoting) {
			autoPivotCommand->Update(currTimeSec, deltaTimeSec);
			if (autoPivotCommand->IsDone()) {
				isPivoting = false;
				printf("Final Angle: %f\n", robot->GetYaw());
			}
		}
	}

	void TestPeriodic()
	{
	}
};

START_ROBOT_CLASS(MainProgram)
