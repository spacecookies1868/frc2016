/*
 * Logger.cpp
 *
 *  Created on: Jan 21, 2016
 *      Author: chloe
 */

#include "Logger.h"

std::string Logger::dataFilePath = "/home/lvuser/datalog.txt";
std::string Logger::actionFilePath = "/home/lvuser/actionlog.txt";
std::ofstream Logger::logData;
std::ofstream Logger::logAction;

void Logger::LogState(RobotModel* myRobot, RemoteControl *myHumanControl) {
	// std::string fileName = dataFilePath + std::to_string(myRobot->GetTime());

	if (!logData.is_open()) {
		logData.open(dataFilePath, std::ofstream::out);
	}
	logData << myRobot->GetTime() <<", " <<
			myRobot->GetWheelSpeed(RobotModel::kLeftWheels) << ", " <<
			myRobot->GetWheelSpeed(RobotModel::kRightWheels) << ", " <<
			myRobot->IsLowGear() << ", " << myRobot->GetVoltage() << ", " <<
			myHumanControl->GetJoystickValue(RemoteControl::kLeftJoy,
					RemoteControl::kX) << ", " << myHumanControl->GetJoystickValue(
							RemoteControl::kLeftJoy, RemoteControl::kY) << ", " <<
			myHumanControl->GetJoystickValue(RemoteControl::kRightJoy,
					RemoteControl::kX) << ", " << myHumanControl->GetJoystickValue(
							RemoteControl::kRightJoy, RemoteControl::kY) << ", " <<
			myHumanControl->GetReverseDriveDesired() << ", " <<
			myHumanControl->GetLowGearDesired() << "\r\n";
}
/* format:
 * robotmodel state / controlboard state
 *
 * ie:
 *
 * timer / left motor / right motor / gear shift / pdp voltage / leftjoy x / leftjoy y
 * 			/ rightjoy x / rightjoy y / reverse desired / gearshift desired /
 */

void Logger::LogAction(RobotModel* myRobot, const std::string& fileName, int line,
		const std::string& stateName, bool state) {
	if (!logAction.is_open()) {
		logAction.open(actionFilePath, std::ofstream::out);
	}
	logAction << myRobot->GetTime() << ", " << fileName << ", " <<
			line << ", " << stateName << ", " << state << "\r\n";
}
