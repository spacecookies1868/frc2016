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
		try {
			lw = LiveWindow::GetInstance();
			robot = new RobotModel();
		} catch (std::exception &e) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += e.what();
			DriverStation::ReportError(err_string.c_str());
		}
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
		Logger::LogState(robot, humanControl);
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

		Logger::LogState(robot, humanControl);

#if USE_CAMERA
		CameraServer::GetInstance()->SetQuality(50); // ?? what is quality?
		CameraServer::GetInstance()->SetImage(robot->GetCameraImage());
#endif
	}

	void TestPeriodic() {
		printf("yaw: %f\n", robot->GetNavXYaw());
		printf("roll: %f\n", robot->GetNavXRoll());
		printf("pitch: %f\n", robot->GetNavXPitch());
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
