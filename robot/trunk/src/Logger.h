/*
 * Logger.h
 *
 *  Created on: Jan 21, 2016
 *      Author: chloe
 *
 *      utility functions for logging state of robot
 *
 *      todo:
 *      -severity-based logging
 *      -add header
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include "RobotModel.h"
#include "RemoteControl.h"
#include <fstream>
#include <string>
#include <ctime>

#define LOG(myRobot, stateName, state) {DO_PERIODIC(1, Logger::LogAction(myRobot, __FILE__, __LINE__, stateName, state))}
#define DUMP(stateName, state) {DO_PERIODIC(1, Logger::LogAction(__FILE__, __LINE__, stateName, state))}
class Logger {
public:
	static void LogState(RobotModel* myRobot, RemoteControl *myHumanControl);
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
