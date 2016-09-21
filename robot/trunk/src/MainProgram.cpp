#include "WPILib.h"
#include "ControlBoard.h"
#include "RobotModel.h"
#include "DriveController.h"
#include "SuperstructureController.h"
#include "AutonomousController.h"
#include "CameraController.h"
#include "PowerController.h"
#include "Debugging.h"
#include "Logger.h"
#include <string.h>

//This file is written on the base class Iterative Robot
//MainProgram calls all the classes in Autonomous or Teleop
class MainProgram : public IterativeRobot {
	//LiveWindow helps in Test mode
	LiveWindow *lw;
	//Creates a robot from class RobotModel
	RobotModel *robot;
	//Creates a human control from RemoteControl, which includes ControlBoard
	RemoteControl *humanControl;
	//Creates a controller for drivetrain, superstructure, autonomous, the camera, and the power
	DriveController *driveController;
	SuperstructureController *superstructureController;
	AutonomousController *autonomousController;
	CameraController *cameraController;
	PowerController *powerController;

	//Creates a time-keeper
	double currTimeSec;
	double lastTimeSec;
	double deltaTimeSec;

public:
	MainProgram(void) {
		try {
			//Defines live window for test mode
			lw = LiveWindow::GetInstance();
			//Define robot
			robot = new RobotModel();
		} catch (std::exception &e) {
			DUMP("NavX Error", "Error Instantiating NavX");
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += e.what();
			DriverStation::ReportError(err_string.c_str());
		}
		//Defines the control board, drive controller, superstructure controller, camera controller, autonomous controller, and power controller
		humanControl = new ControlBoard();
		driveController = new DriveController(robot, humanControl);
		superstructureController = new SuperstructureController(robot, humanControl);
		cameraController = new CameraController(robot);
		autonomousController = new AutonomousController(robot, driveController, superstructureController, cameraController, humanControl);
		powerController = new PowerController(robot, humanControl);

		//Initializes timekeeper variables
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
	}

private:
	//Initializes robot
	void RobotInit() {
		robot->ResetTimer();
		robot->Reset();
#if USE_USB_CAMERA
		//CameraServer::GetInstance()->SetQuality(50);
		//the camera name (ex "cam1") can be found through the roborio web interface
		//CameraServer::GetInstance()->StartAutomaticCapture("cam1");
#endif
//		CameraServer::GetInstance()->SetQuality(50);
//		//the camera name (ex "cam1") can be found through the roborio web interface
//		CameraServer::GetInstance()->StartAutomaticCapture("cam1");
		RefreshAllIni();
	}

	//Initializes Autonomous
	void AutonomousInit() {
		RefreshAllIni();
		robot->ResetTimer();
		robot->ZeroNavXYaw();
		robot->Reset();
		robot->SetBrakeOff();
		//Resets all controllers
		driveController->Reset();
		superstructureController->Reset();
		cameraController->Reset();
		autonomousController->Reset();

		//printf("Auto Intake Up: %i\n", superstructureController->autoIntakeUp);
		//Resets timer variables
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
		//Starts Autonomous
		autonomousController->StartAutonomous();
		//printf("Auto Intake Up: %i\n", superstructureController->autoIntakeUp);
		Wait(0.5);
	}

	//Autonomous periodic is what happens iteratively in autonomous
	void AutonomousPeriodic() {
		//printf("Auto Intake Up: %i\n", superstructureController->autoIntakeUp);
		//Timer is updated
		lastTimeSec = currTimeSec;
		currTimeSec = robot->GetTime();
		deltaTimeSec = currTimeSec - lastTimeSec;
		//Autonomous is updated
		autonomousController->Update(currTimeSec, deltaTimeSec);
		Logger::LogState(robot, humanControl);

		//printf("Auto Intake Up: %i\n", superstructureController->autoIntakeUp);
#if USE_USB_CAMERA
		if (robot->usbCamera->GetError().GetCode() == 0) {
			CameraServer::GetInstance()->SetQuality(20);
			CameraServer::GetInstance()->SetImage(robot->GetCameraImage());
		} else {

		}
#endif

	}

	//Initializes Teleop
	void TeleopInit() {
		RefreshAllIni();
		robot->ResetTimer();

		//Resets all controllers
		robot->Reset();
		driveController->Reset();
		superstructureController->Reset();
		cameraController->Reset();
		autonomousController->Reset();
		powerController->Reset();

		//Resets timer variables
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
	}

	//Teleop periodic happens iteratively during teleop
	void TeleopPeriodic() {
		//Updates timer
		lastTimeSec = currTimeSec;
		currTimeSec = robot->GetTime();
		deltaTimeSec = currTimeSec - lastTimeSec;

		//Reads controls and updates controllers accordingly
		humanControl->ReadControls();
		robot->UpdateCurrent();
		driveController->Update(currTimeSec, deltaTimeSec);
		superstructureController->Update(currTimeSec, deltaTimeSec);
		//Checks voltage of battery
		if (robot->GetVoltage() < 9.5) {
			printf("LOW VOLTS LOW VOLTS LOW VOLTS LOW VOLTS LOW VOLTS LOW VOLTS \n");
		}

		//printf("Right Encoder Val %f\n", robot->GetRightEncoderVal());
		//printf("Left Encoder Val %f\n", robot->GetLeftEncoderVal());
		powerController->Update(currTimeSec, deltaTimeSec);
		//Logs state of robot
		Logger::LogState(robot, humanControl);
		LOG(robot, "Right Current Draw A", robot->GetCurrent(2));
		LOG(robot, "Right Current Draw B", robot->GetCurrent(1));
		LOG(robot, "Left Current Draw A", robot->GetCurrent(7));
		LOG(robot, "Left Current Draw B", robot->GetCurrent(8));
//Only executes if camera is plugged in and variable is true
#if USE_CAMERA
		//Sets quality and gets camera image
		CameraServer::GetInstance()->SetQuality(30); // ?? what is quality?
		CameraServer::GetInstance()->SetImage(robot->GetCameraImage());
#endif
#if USE_USB_CAMERA
		if (robot->usbCamera->GetError().GetCode() == 0) {
			CameraServer::GetInstance()->SetQuality(20);
			CameraServer::GetInstance()->SetImage(robot->GetCameraImage());
		} else {

		}
#endif
	}

	//Test Periodic happens iteratively in test mode and is used to check states of various robot components
	void TestPeriodic() {
#if USE_USB_CAMERA
		if (robot->usbCamera->GetError().GetCode() == 0) {
			CameraServer::GetInstance()->SetQuality(20);
			CameraServer::GetInstance()->SetImage(robot->GetCameraImage());
		} else {

		}
#endif

		DO_PERIODIC(10, LOG(robot, "Navx angle", robot->GetNavXYaw()));
		DO_PERIODIC(10, LOG(robot, "Navx pitch", robot->GetNavXPitch()));
		DO_PERIODIC(10, LOG(robot, "Navx Roll", robot->GetNavXRoll()));
		DO_PERIODIC(10, LOG(robot, "Left Encoder", robot->GetLeftEncoderVal()));
		DO_PERIODIC(10, LOG(robot, "Right Encoder", robot->GetRightEncoderVal()));
		DO_PERIODIC(10, LOG(robot, "Left Joy Y",
						humanControl->GetJoystickValue(RemoteControl::kLeftJoy, RemoteControl::kY)));
		DO_PERIODIC(10, LOG(robot, "Left Joy X",
						humanControl->GetJoystickValue(RemoteControl::kLeftJoy, RemoteControl::kX)));
		DO_PERIODIC(10, LOG(robot, "Right Joy Y",
						humanControl->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kY)));
		DO_PERIODIC(10, LOG(robot, "Right Joy X",
						humanControl->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kX)));
		DO_PERIODIC(10, LOG(robot, "Reverse Drive Desired", humanControl->GetReverseDriveDesired()));
		DO_PERIODIC(10, LOG(robot, "Arcade Drive Desired", humanControl->GetArcadeDriveDesired()));
		DO_PERIODIC(10, printf("Intake switch state %d\n", robot->GetIntakeSwitchState()));
		DO_PERIODIC(10, printf("Left Encoder %f\n", robot->GetLeftEncoderVal()));
		DO_PERIODIC(10, printf("Right Encoder %f\n", robot->GetRightEncoderVal()));

		//DO_PERIODIC(10, printf("Navx angle %f\n", robot->GetNavXYaw()));
		/*DO_PERIODIC(10, printf("Navx pitch %f\n", robot->GetNavXPitch()));
		DO_PERIODIC(10, printf("Navx Roll %f\n", robot->GetNavXRoll()));
		DO_PERIODIC(10, printf("Left Encoder %f\n", robot->GetLeftEncoderVal()));
		DO_PERIODIC(10, printf("Right Encoder %f\n", robot->GetRightEncoderVal()));
		DO_PERIODIC(10, printf("Pressure %f\n", robot->GetPressureSensorVal()));*/
	//	DO_PERIODIC(10, printf("Left Joy Y %f\n",
	//					humanControl->GetJoystickValue(RemoteControl::kLeftJoy, RemoteControl::kY)));
	//	DO_PERIODIC(10, printf("Left Joy X %f\n",
	//					humanControl->GetJoystickValue(RemoteControl::kLeftJoy, RemoteControl::kX)));
	//	DO_PERIODIC(10, printf("Right Joy Y %f\n",
	//					humanControl->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kY)));
	//	DO_PERIODIC(10, printf("Right Joy X %f\n",
	//					humanControl->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kY)));
		//DO_PERIODIC(10, printf("Reverse Drive Desired %f\n", humanControl->GetReverseDriveDesired()));
		//DO_PERIODIC(10, printf("Arcade Drive Desired %f\n", humanControl->GetArcadeDriveDesired()));


	}
	//Initializes when robot is disabled
	void DisabledInit() {
		robot->Reset();
		driveController->Reset();
		superstructureController->Reset();
		autonomousController->Reset();
		cameraController->Reset();
		powerController->Reset();
		Logger::CloseLogs();
	}

	//Disabled Periodic happens periodically when the robot is disabled
	void DisabledPeriodic() {

#if USE_USB_CAMERA
	if (robot->usbCamera->GetError().GetCode() == 0) {
		CameraServer::GetInstance()->SetQuality(20);
		CameraServer::GetInstance()->SetImage(robot->GetCameraImage());
	} else {
		printf("Camera unplugged");
	}
#endif

	}

	//Refreshes ini files
	void RefreshAllIni() {
		robot->RefreshIni();
		autonomousController->RefreshIni();
		driveController->RefreshIni();
		superstructureController->RefreshIni();
		powerController->RefreshIni();
	}

};

START_ROBOT_CLASS(MainProgram);
