#include "WPILib.h"
#include "ControlBoard.h"
#include "RobotModel.h"
#include "DriveController.h"
#include "AutonomousController.h"
#include "Debugging.h"
#include <string.h>

class MainProgram : public IterativeRobot {
	LiveWindow *lw;
	RobotModel *robot;
	ControlBoard *humanControl;
	DriveController *driveController;
	AutonomousController *autonomousController;

	double currTimeSec;
	double lastTimeSec;
	double deltaTimeSec;

public:
	MainProgram(void) {
		lw = LiveWindow::GetInstance();
		robot = new RobotModel();
		humanControl = new ControlBoard();
		driveController = new DriveController(robot, humanControl);

		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
	}

private:
	void RobotInit() {
		robot->EnableCompressor();
		robot->ResetTimer();
		robot->ResetGyro();
		robot->ResetDriveEncoders();
		printf("robot init\n");
		driveController->RefreshIni();
		autonomousController->RefreshIni();
		//elevatorController->RefreshIni();
	}

	void AutonomousInit() {
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
		robot->ResetDriveEncoders();
		RefreshAllIni();
		autonomousController->StartAutonomous();
		printf("auto init\n");
	}

	void AutonomousPeriodic() {
		lastTimeSec = currTimeSec;
		currTimeSec = robot->timer->Get();
		deltaTimeSec = currTimeSec - lastTimeSec;

		autonomousController->Update(currTimeSec, deltaTimeSec);
		//printf("auto periodic\n");
	}

	void TeleopInit() {
		RefreshAllIni();

		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;

		driveController->Reset();
		autonomousController->Reset();
		robot->ResetDriveEncoders();

		driveController->RefreshIni();
		autonomousController->RefreshIni();
	}

	void TeleopPeriodic() {
		lastTimeSec = currTimeSec;
		currTimeSec = robot->timer->Get();
		deltaTimeSec = currTimeSec - lastTimeSec;

		humanControl->ReadControls();
		driveController->Update(currTimeSec, deltaTimeSec);

		if (robot->GetVoltage() < 9.5) {
			printf("LOW VOLTS LOW VOLTS LOW VOLTS LOW VOLTS LOW VOLTS LOW VOLTS \n");
		}
	}

	void TestPeriodic() {
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		printf("fleft encoder: %f\n", robot->GetFrontLeftEncoderVal());
		printf("fright encoder: %f\n", robot->GetFrontRightEncoderVal());
		printf("rleft encoder: %f\n", robot->GetRearLeftEncoderVal());
		printf("rright encoder: %f\n", robot->GetRearRightEncoderVal());
	}

	void DisabledInit() {
		robot->DisableCompressor();
		driveController->Reset();
		autonomousController->Reset();
	}

	void RefreshAllIni() {
		robot->RefreshIniFile();
		autonomousController->RefreshIni();
		driveController->RefreshIni();
	}
};

START_ROBOT_CLASS(MainProgram);