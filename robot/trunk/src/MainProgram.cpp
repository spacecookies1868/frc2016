#include "WPILib.h"
#include "ControlBoard.h"
#include "RobotModel.h"
#include "DriveController.h"
#include "SuperstructureController.h"
#include "AutonomousController.h"
#include "CameraController.h"
#include "Debugging.h"
#include "Logger.h"
#include <string.h>

class MainProgram : public IterativeRobot {
	LiveWindow *lw;
	RobotModel *robot;
	RemoteControl *humanControl;
	DriveController *driveController;
	SuperstructureController *superstructureController;
	AutonomousController *autonomousController;
	CameraController *cameraController;

	double currTimeSec;
	double lastTimeSec;
	double deltaTimeSec;

public:
	MainProgram(void) {
		lw = LiveWindow::GetInstance();
		robot = new RobotModel();
		humanControl = new ControlBoard();
		driveController = new DriveController(robot, humanControl);
		superstructureController = new SuperstructureController(robot, humanControl);
		cameraController = new CameraController(robot);
		autonomousController = new AutonomousController(robot, driveController, superstructureController, cameraController);

		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
	}

private:
	void RobotInit() {
		robot->ResetTimer();
		//robot->ResetDriveEncoders();
		RefreshAllIni();
	}

	void AutonomousInit() {
		RefreshAllIni();
		robot->ResetTimer();

		robot->Reset();
		driveController->Reset();
		superstructureController->Reset();
		cameraController->Reset();
		autonomousController->Reset();

		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
//		robot->ResetDriveEncoders();
		autonomousController->StartAutonomous();
	}

	void AutonomousPeriodic() {
		lastTimeSec = currTimeSec;
		currTimeSec = robot->GetTime();
		deltaTimeSec = currTimeSec - lastTimeSec;
		autonomousController->Update(currTimeSec, deltaTimeSec);
	}

	void TeleopInit() {
		RefreshAllIni();
		robot->ResetTimer();

		robot->Reset();
		driveController->Reset();
		superstructureController->Reset();
		cameraController->Reset();
		autonomousController->Reset();

		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
	}

	void TeleopPeriodic() {
		lastTimeSec = currTimeSec;
		currTimeSec = robot->GetTime();
		deltaTimeSec = currTimeSec - lastTimeSec;

		humanControl->ReadControls();
		driveController->Update(currTimeSec, deltaTimeSec);
		superstructureController->Update(currTimeSec, deltaTimeSec);

		if (robot->GetVoltage() < 9.5) {
			printf("LOW VOLTS LOW VOLTS LOW VOLTS LOW VOLTS LOW VOLTS LOW VOLTS \n");
		}
	}

	void TestPeriodic() {
		printf("fleft encoder: %f\n", robot->GetLeftEncoderVal());
		printf("fright encoder: %f\n", robot->GetRightEncoderVal());
		LOG(robot, "Yaw values: ", robot->GetNavXYaw());
	}

	void DisabledInit() {
		robot->Reset();
		driveController->Reset();
		superstructureController->Reset();
		autonomousController->Reset();
		cameraController->Reset();
	}

	void DisabledPeriodic() {
	}

	void RefreshAllIni() {
		robot->RefreshIni();
		autonomousController->RefreshIni();
		driveController->RefreshIni();
		superstructureController->RefreshIni();
	}

};

START_ROBOT_CLASS(MainProgram);
