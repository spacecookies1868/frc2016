/*
 * Logger.cpp
 *
 *  Created on: Jan 21, 2016
 *      Author: chloe
 */

#include "Logger.h"

std::ofstream Logger::logData;
std::ofstream Logger::logAction;

void Logger::LogState(RobotModel* myRobot, RemoteControl *myHumanControl) {
	if (!logData.is_open()) {
		logData.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_datalog.txt")).c_str()), std::ofstream::out);
		logData << "Time, Left Encoder, Right Encoder, Left Wheel Speed,"
				<< "Right Wheel Speed, Yaw, Roll, Pitch, Voltage, Pressure, "
				<< "Outtake Encoder, Outtake Motor Speed, Intake Motor Speed, "
				<< "Intake Down, Defense Down, Low Gear, Left Joy X, Left Joy Y, "
				<< "Right Joy X, Right Joy Y, Reverse, Arcade, Defense Desired, "
				<< "Intake Reverse Desired, Intake Forward Desired, Intake Piston Desired, "
				<< "Low Gear Desired, Outtake Desired, Quick Turn Desired \r\n";
	}
	logData << myRobot->GetTime() <<", " <<
			myRobot->GetLeftEncoderVal() << ", " <<
			myRobot->GetRightEncoderVal() << ", " <<
			myRobot->GetWheelSpeed(RobotModel::kLeftWheels) << ", " <<
			myRobot->GetWheelSpeed(RobotModel::kRightWheels) << ", " <<
			myRobot->GetNavXYaw() << ", " <<
			myRobot->GetNavXRoll() << ", " <<
			myRobot->GetNavXPitch() << ", " <<
			myRobot->GetVoltage() << ", " <<
			myRobot->GetPressureSensorVal() << ", " <<
			myRobot->GetOuttakeEncoderVal() << ", " <<
			myRobot->GetOuttakeMotorSpeed() << ", " <<
			myRobot->GetIntakeMotorSpeed() << ", " <<
			myRobot->IsIntakeArmDown() << ", " <<
			myRobot->IsDefenseManipDown() << ", " <<
			myRobot->IsLowGear() << ", " <<
			myHumanControl->GetJoystickValue(RemoteControl::kLeftJoy, RemoteControl::kX) << ", " <<
			myHumanControl->GetJoystickValue(RemoteControl::kLeftJoy, RemoteControl::kY) << ", " <<
			myHumanControl->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kX) << ", " <<
			myHumanControl->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kY) << ", " <<
			myHumanControl->GetReverseDriveDesired() << ", " <<
			myHumanControl->GetArcadeDriveDesired() << ", " <<
			myHumanControl->GetDefenseManipDesired() << ", " <<
			myHumanControl->GetIntakeMotorReverseDesired() << ", " <<
			myHumanControl->GetIntakeMotorForwardDesired()  << ", " <<
			myHumanControl->GetIntakePistonDesired()  << ", " <<
			myHumanControl->GetLowGearDesired() << ", " <<
			myHumanControl->GetOuttakeDesired()  << ", " <<
			myHumanControl->GetQuickTurnDesired() << "\r\n";
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
