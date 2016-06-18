#include "AutoCommand.h"
#include <math.h>
#include "ini.h"
#include <iostream>
#include <string>
#include "Logger.h"
#include "Debugging.h"

#define PI 3.14159265358979

/*
 * Waiting Command: Robot waits. Takes in a double in seconds.
 */

WaitingCommand::WaitingCommand(double myWaitTimeSec) {
	waitTimeSec = myWaitTimeSec;
	timer = new Timer();
	isDone = false;
}

void WaitingCommand::Init() {
	timer->Start();
}

void WaitingCommand::Update(double currTimeSec, double deltaTimeSec) {
	isDone = (timer->Get() >= waitTimeSec);
	if(isDone) {
		DUMP("done! :)", currTimeSec);
	}
}

bool WaitingCommand::IsDone() {
	return isDone;
}


/*
 * Diagnostic Command: Automatic functionality test.
 */

DiagnosticCommand::DiagnosticCommand(RobotModel* myRobot, SuperstructureController* mySuperstructure) {
	robot = myRobot;
	superstructure = mySuperstructure;
	isDone = false;
	m_stateVal = kInit;
	nextState = kInit;
	timeCount = 0.0;
	waitTime = 3.0;
}

void DiagnosticCommand::Init() {
	m_stateVal = kInit;
}

void DiagnosticCommand::Update(double currTimeSec, double deltaTimeSec) {
	switch(m_stateVal) {
	case(kInit) : {
		nextState = kLeftMotorA;
		break;
	}
	case (kLeftMotorA) : {
		timeCount += deltaTimeSec;
		if (timeCount < waitTime) {
			printf("LEFT POSITIVE A \n");
			robot->leftDriveMotorA->SetSpeed(0.5);
		} else if (waitTime <= timeCount && timeCount < 2*waitTime) {
			printf("LEFT NEGATIVE A \n");
			robot->leftDriveMotorA->SetSpeed(-0.5);
		} else {
			robot->leftDriveMotorA->SetSpeed(0.0);
			timeCount = 0.0;
			nextState = kLeftMotorB;
		}
		break;
	}
	case (kLeftMotorB) : {
		timeCount += deltaTimeSec;
		if (timeCount < waitTime) {
			printf("LEFT POSITIVE B \n");
			robot->leftDriveMotorB->SetSpeed(0.5);
		} else if (waitTime <= timeCount && timeCount < 2*waitTime) {
			printf("LEFT NEGATIVE B \n");
			robot->leftDriveMotorB->SetSpeed(-0.5);
		} else {
			robot->leftDriveMotorB->SetSpeed(0.0);
			timeCount = 0.0;
			nextState = kRightMotorA;
		}
		break;
	}
	case (kRightMotorA) : {
		timeCount += deltaTimeSec;
		if (timeCount < waitTime) {
			printf("RIGHT POSITIVE A \n");
			robot->rightDriveMotorA->SetSpeed(0.5);
		} else if (waitTime <= timeCount && timeCount < 2*waitTime) {
			printf("RIGHT NEGATIVE A \n");
			robot->rightDriveMotorA->SetSpeed(-0.5);
		} else {
			robot->rightDriveMotorA->SetSpeed(0.0);
			timeCount = 0.0;
			nextState = kRightMotorB;
		}
		break;
	}
	case (kRightMotorB): {
		timeCount += deltaTimeSec;
		if (timeCount < waitTime) {
			printf("RIGHT POSITIVE B \n");
			robot->rightDriveMotorB->SetSpeed(0.5);
		} else if (waitTime <= timeCount && timeCount < 2*waitTime) {
			printf("RIGHT NEGATIVE B \n");
			robot->rightDriveMotorB->SetSpeed(-0.5);
		} else {
			robot->rightDriveMotorB->SetSpeed(0.0);
			timeCount = 0.0;
			nextState = kLeftMotor;
		}
		break;
	}
	case (kLeftMotor): {
		timeCount += deltaTimeSec;
		if (timeCount < waitTime) {
			printf("LEFT SIDE FORWARD \n");
			robot->SetWheelSpeed(RobotModel::kLeftWheels, 0.5);
		} else if (waitTime <= timeCount && timeCount < 2*waitTime) {
			printf("LEFT SIDE BACKWARD \n");
			robot->SetWheelSpeed(RobotModel::kLeftWheels, -0.5);
		} else {
			robot->SetWheelSpeed(RobotModel::kLeftWheels, 0.0);
			timeCount = 0.0;
			nextState = kRightMotor;
		}
		break;
	}
	case (kRightMotor): {
		timeCount += deltaTimeSec;
		if (timeCount < waitTime) {
			printf("RIGHT SIDE FORWARD \n");
			robot->SetWheelSpeed(RobotModel::kRightWheels, 0.5);
		} else if (waitTime <= timeCount && timeCount < 2*waitTime) {
			printf("RIGHT SIDE BACKWARD \n");
			robot->SetWheelSpeed(RobotModel::kRightWheels, -0.5);
		} else {
			robot->SetWheelSpeed(RobotModel::kRightWheels, 0.0);
			timeCount = 0.0;
			nextState = kAllMotors;
		}
		break;
	}
	case (kAllMotors): {
		timeCount += deltaTimeSec;
		if (timeCount < waitTime) {
			printf("FORWARD \n");
			robot->SetWheelSpeed(RobotModel::kAllWheels, 0.5);
		} else if (waitTime <= timeCount && timeCount < 2*waitTime) {
			printf("BACKWARD \n");
			robot->SetWheelSpeed(RobotModel::kAllWheels, -0.5);
		} else {
			robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
			timeCount = 0.0;
			nextState = kIntakeMotor;
		}
		break;
	}
	case (kIntakeMotor): {
		timeCount += deltaTimeSec;
		if (timeCount < waitTime) {
			printf("INTAKE FORWARD \n");
			robot->SetIntakeMotorSpeed(0.5);
		} else if (waitTime <= timeCount && timeCount < 2*waitTime) {
			printf("INTAKE BACKWARD \n");
			robot->SetIntakeMotorSpeed(-0.5);
		} else {
			robot->SetIntakeMotorSpeed(0.0);
			timeCount = 0.0;
			nextState = kOuttakeMotor;
		}
		break;
	}
	case (kOuttakeMotor): {
		timeCount += deltaTimeSec;
		if (timeCount < waitTime) {
			printf("OUTTAKE FORWARD \n");
			robot->SetOuttakeMotorSpeed(0.5);
		} else if (waitTime <= timeCount && timeCount < 2*waitTime) {
			printf("OUTTAKE BACKWARD \n");
			robot->SetOuttakeMotorSpeed(-0.5);
		} else {
			robot->SetOuttakeMotorSpeed(0.0);
			timeCount = 0.0;
			nextState = kIntakeDown;
		}
		break;
	}
	case (kIntakeDown): {
		timeCount += deltaTimeSec;
		if (timeCount < waitTime) {
			superstructure->SetAutoIntakeDown(true);
		} else {
			superstructure->SetAutoIntakeDown(false);
			timeCount = 0.0;
			nextState = kIntakeUp;
		}
		break;
	}
	case (kIntakeUp): {
		timeCount += deltaTimeSec;
		if (timeCount < waitTime) {
			superstructure->SetAutoIntakeUp(true);
		} else {
			superstructure->SetAutoIntakeUp(false);
			timeCount = 0.0;
			nextState = kDefenseManipDown;
		}
		break;
	}
	case (kDefenseManipDown): {
		timeCount += deltaTimeSec;
		if (timeCount < waitTime) {
			superstructure->SetAutoDefenseManipDown(true);
		} else {
			superstructure->SetAutoDefenseManipDown(false);
			timeCount = 0.0;
			nextState = kDefenseManipUp;
		}
		break;
	}
	case (kDefenseManipUp): {
		timeCount += deltaTimeSec;
		if (timeCount < waitTime) {
			superstructure->SetAutoDefenseManipUp(true);
		} else {
			superstructure->SetAutoDefenseManipUp(false);
			timeCount = 0.0;
			nextState = kDefenseManipUp;
		}
		break;
	}
	}
	m_stateVal = nextState;
}

bool DiagnosticCommand::IsDone() {
	return isDone;
}
