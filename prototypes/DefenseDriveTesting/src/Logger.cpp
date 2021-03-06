/*
 * Logger.cpp
 *
 *  Created on: Jan 21, 2016
 *      Author: chloe
 */

#include "Logger.h"

std::ofstream Logger::logData;
std::ofstream Logger::logAction;

void Logger::LogState(RobotModel* myRobot, DefenseDrive* myDefenseDrive) {
	if (!logData.is_open()) {
		logData.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_datalog.txt")).c_str()), std::ofstream::out);
		logData << "Time, DefenseDriveState, LeftWheelSpeed, RightWheelSpeed, Yaw, Roll, Pitch\r\n";
	}
	logData <<
			myRobot->GetTime() << ", " <<
			myDefenseDrive->GetState() << ", " <<
			myRobot->GetWheelSpeed(RobotModel::kLeftWheels) << ", " <<
			myRobot->GetWheelSpeed(RobotModel::kRightWheels) << ", " <<
			myRobot->GetYaw() << ", " <<
			myRobot->GetRoll() << ", " <<
			myRobot->GetPitch() << "\r\n";
	logData.flush();
}

void Logger::LogState(RobotModel* myRobot, RockWallDrive* myRockWallDrive) {
	if (!logData.is_open()) {
		logData.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_datalog.txt")).c_str()), std::ofstream::out);
		logData << "Time, DefenseDriveState, LeftWheelSpeed, RightWheelSpeed, Yaw, Roll, Pitch\r\n";
	}
	logData <<
			myRobot->GetTime() << ", " <<
			myRockWallDrive->GetState() << ", " <<
			myRobot->GetWheelSpeed(RobotModel::kLeftWheels) << ", " <<
			myRobot->GetWheelSpeed(RobotModel::kRightWheels) << ", " <<
			myRobot->GetYaw() << ", " <<
			myRobot->GetRoll() << ", " <<
			myRobot->GetPitch() << "\r\n";
	logData.flush();
}

void Logger::LogState(RobotModel* myRobot, MoatDrive* myMoatDrive) {
	if (!logData.is_open()) {
		logData.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_datalog.txt")).c_str()), std::ofstream::out);
		logData << "Time, DefenseDriveState, LeftWheelSpeed, RightWheelSpeed, Yaw, Roll, Pitch\r\n";
	}
	logData <<
			myRobot->GetTime() << ", " <<
			myMoatDrive->GetState() << ", " <<
			myRobot->GetWheelSpeed(RobotModel::kLeftWheels) << ", " <<
			myRobot->GetWheelSpeed(RobotModel::kRightWheels) << ", " <<
			myRobot->GetYaw() << ", " <<
			myRobot->GetRoll() << ", " <<
			myRobot->GetPitch() << "\r\n";
	logData.flush();
}
/* format:
 * robotmodel state / controlboard state
 *
 * ie:
 *
 * timer / left motor / right motor / gear shift / pdp voltage / leftjoy x / leftjoy y
 * 			/ rightjoy x / rightjoy y / reverse desired / gearshift desired /
 */

/* overloaded methods with time stamp */

void Logger::LogAction(RobotModel* myRobot, const std::string& fileName, int line,
				const std::string& stateName, double state) {
	if (!logAction.is_open()) {
			logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out);
	}
	logAction << myRobot->GetTime() << ", " << fileName << ", " << line << ", " << stateName
			<< ", " << state << "\r\n";
	logAction.flush();
}

void Logger::LogAction(RobotModel* myRobot, const std::string& fileName, int line,
				const std::string& stateName, const std::string& state) {
	if (!logAction.is_open()) {
			logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out);
	}
	logAction << myRobot->GetTime() << ", " << fileName << ", " << line << ", " << stateName
			<< ", " << state << "\r\n";
	logAction.flush();
}

/* overloaded methods without time stamp */
void Logger::LogAction(const std::string& fileName, int line, const std::string& stateName,
			bool state) {
	if (!logAction.is_open()) {
		logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out);
	}
	logAction << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
	logAction.flush();
}
void Logger::LogAction(const std::string& fileName, int line, const std::string& stateName,
			double state) {
	if (!logAction.is_open()) {
		logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out);
	}
	logAction << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
	logAction.flush();
}
void Logger::LogAction(const std::string& fileName, int line, const std::string& stateName,
			const std::string& state) {
	if (!logAction.is_open()) {
		logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out);
	}
	logAction << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
	logAction.flush();
}

std::string Logger::GetTimeStamp(const char* fileName) {
/*	struct timespec tp;
	clock_gettime(CLOCK_REALTIME,&tp);
	double realTime = (double)tp.tv_sec + (double)((double)tp.tv_nsec*1e-9);
*/
	time_t rawtime = time(0);
	struct tm * timeinfo;
	char buffer [80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);
	strftime (buffer, 80, fileName, timeinfo); // fileName contains %F_%H_%M

	return buffer;
}
