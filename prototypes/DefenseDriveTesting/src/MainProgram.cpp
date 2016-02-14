#include <WPILib.h>
#include <AHRS.h>
#include "RobotModel.h"
#include "DefenseDrive.h"

class MainProgram: public IterativeRobot
{

public:
	RobotModel* robot;
	DefenseDrive* defenseDrive;
	Timer* timer;
	Compressor* compressor;
	double currTimeSec, lastTimeSec, deltaTimeSec;

	MainProgram() {
		robot = new RobotModel();
		timer = new Timer();
		compressor = new Compressor();
	}

	void RobotInit()
	{

	}

	void AutonomousInit()
	{
		defenseDrive = new DefenseDrive(robot);
		defenseDrive->Init();
		timer->Reset();
		timer->Start();
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
	}

	void AutonomousPeriodic()
	{
//		lastTimeSec = currTimeSec;
//		currTimeSec = timer->Get();
//		deltaTimeSec = currTimeSec - lastTimeSec;
//
//		if (currTimeSec < 2.1) {
//			robot->SetWheelSpeed(RobotModel::kAllWheels, 0.6);
//		} else {
//			robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
//		}
//		SmartDashboard::PutNumber("Yaw: \n", robot->GetYaw());
//		SmartDashboard::PutNumber("Pitch: \n", robot->GetPitch());
//		SmartDashboard::PutNumber("Roll: \n", robot->GetRoll());
		if (defenseDrive->IsDone()) {
			robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		} else {
			defenseDrive->Update(currTimeSec, deltaTimeSec);
		}
		SmartDashboard::PutNumber("Roll: %f\n", robot->GetRoll());
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
		SmartDashboard::PutNumber("Roll: %f\n", robot->GetRoll());
	}
};

START_ROBOT_CLASS(MainProgram)
