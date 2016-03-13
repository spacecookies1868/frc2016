#ifndef LOGGER_H_
#define LOGGER_H_

#include "RobotModel.h"
#include "DefenseDrive.h"
#include "RockWallDrive.h"
#include "MoatDrive.h"
#include <fstream>
#include <string>
#include <ctime>

#define LOG(myRobot, stateName, state) {DO_PERIODIC(1, Logger::LogAction(myRobot, __FILE__, __LINE__, stateName, state))}
#define DUMP(stateName, state) {DO_PERIODIC(1, Logger::LogAction(__FILE__, __LINE__, stateName, state))}
class Logger {
public:
	static void LogState(RobotModel* myRobot, DefenseDrive* myDefenseDrive);
	static void LogState(RobotModel* myRobot, RockWallDrive* myRockWallDrive);
	static void LogState(RobotModel* myRobot, MoatDrive* myRockWallDrive);

	/* with time stamp */
	static void LogAction(RobotModel* myRobot, const std::string& fileName, int line,
				const std::string& stateName, double state);
	static void LogAction(RobotModel* myRobot, const std::string& fileName, int line,
				const std::string& stateName, const std::string& state);
	/* without time stamp */
	static void LogAction(const std::string& fileName, int line, const std::string& stateName,
			bool state);
	static void LogAction(const std::string& fileName, int line, const std::string& stateName,
			double state);
	static void LogAction(const std::string& fileName, int line, const std::string& stateName,
			const std::string& state);

	static std::string GetTimeStamp(const char* fileName);

private:
	static std::ofstream logData, logAction;
};

#endif /* SRC_LOGGER_H_ */
