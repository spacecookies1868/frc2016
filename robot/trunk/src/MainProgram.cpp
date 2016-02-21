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
		LOG(robot, "Initing", 0.0);
		robot->ResetTimer();
		robot->Reset();
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
		autonomousController->StartAutonomous();
	}

	void AutonomousPeriodic() {
		lastTimeSec = currTimeSec;
		currTimeSec = robot->GetTime();
		deltaTimeSec = currTimeSec - lastTimeSec;
		autonomousController->Update(currTimeSec, deltaTimeSec);
		LOG(robot, "Left Encoder Val", robot->GetLeftEncoderVal());
		LOG(robot, "Right Encoder Val", robot->GetRightEncoderVal());
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
		//LOG(robot, "Updating driveController", 0.0);
		superstructureController->Update(currTimeSec, deltaTimeSec);

		if (robot->GetVoltage() < 9.5) {
			printf("LOW VOLTS LOW VOLTS LOW VOLTS LOW VOLTS LOW VOLTS LOW VOLTS \n");
		}
		printf("Navx val: %f\n", robot->GetNavXYaw());
	}

	void TestPeriodic() {
		//printf("left encoder: %f\n", robot->GetLeftEncoderVal());
		//printf("right encoder: %f\n", robot->GetRightEncoderVal());
		//printf("Pressure sensor value %f\n", robot->GetPressureSensorVal());
		LOG(robot, "Yaw values: ", robot->GetNavXYaw());
	}

	void DisabledInit() {
		robot->Reset();
		driveController->Reset();
		superstructureController->Reset();
		autonomousController->Reset();
		cameraController->Reset();
		//LOG(robot, "Finished disabled init", 0.0);
	}

	void DisabledPeriodic() {
		//LOG(robot, "I'm disabled!", 0.0);
	}

	void RefreshAllIni() {
		robot->RefreshIni();
		autonomousController->RefreshIni();
		driveController->RefreshIni();
		superstructureController->RefreshIni();
	}

};

START_ROBOT_CLASS(MainProgram);
