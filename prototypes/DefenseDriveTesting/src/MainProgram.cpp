#include <WPILib.h>
#include <AHRS.h>
#include "RobotModel.h"
#include "DefenseDrive.h"
#include "RockWallDrive.h"
#include "MoatDrive.h"
#include "Logger.h"

class MainProgram: public IterativeRobot {

public:
	RobotModel* robot;
	//DefenseDrive* defenseDrive;
	//RockWallDrive* rockWallDrive;
	MoatDrive* moatDrive;
	double currTimeSec, lastTimeSec, deltaTimeSec;
	Compressor* compressor;

	MainProgram() {
		robot = new RobotModel();
		//defenseDrive = new DefenseDrive(robot);
		//rockWallDrive = new RockWallDrive(robot);
		moatDrive = new MoatDrive(robot);
		compressor = new Compressor(1);
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
		printf("main program\n");
	}

	void RobotInit()
	{

	}

	void AutonomousInit()
	{
		//defenseDrive->Init();
		//rockWallDrive->Init();
		moatDrive->Init();
	}

	void AutonomousPeriodic()
	{
		lastTimeSec = currTimeSec;
		currTimeSec = robot->GetTime();
		deltaTimeSec = currTimeSec - lastTimeSec;

//		if (defenseDrive->IsDone()) {
//			robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
//		} else {
//			defenseDrive->Update(currTimeSec, deltaTimeSec);
//			Logger::LogState(robot, defenseDrive);
//		}
//		if (rockWallDrive->IsDone()) {
//			robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
//		} else {
//			rockWallDrive->Update(currTimeSec, deltaTimeSec);
//			Logger::LogState(robot, rockWallDrive);
//		}
		if (moatDrive->IsDone()) {
			robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		} else {
			moatDrive->Update(currTimeSec, deltaTimeSec);
			Logger::LogState(robot, moatDrive);
		}
	}

	void TeleopInit()
	{
		compressor->Start();
	}

	void TeleopPeriodic()
	{
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
	}

	void TestInit() {
	}

	void TestPeriodic()
	{
//		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
//		printf("Yaw: %f\n", robot->GetYaw());
//		printf("Roll: %f\n", robot->GetRoll());
//		printf("Pitch: %f\n", robot->GetPitch());
//		SmartDashboard::PutNumber("Roll: %f\n", robot->GetRoll());
	}
};

START_ROBOT_CLASS(MainProgram)
