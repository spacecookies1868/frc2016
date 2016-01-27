/*
 * Logger.h
 *
 *  Created on: Jan 21, 2016
 *      Author: chloe
 *
 *      utility functions for logging state of robot
 *
 *      todo:
 *      -make distinct file names (date ?)
 *      -severity-based logging
 *      -slow down frequency of logging
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include "RobotModel.h"
#include "RemoteControl.h"
#include <fstream>
#include <string>
#include <ctime>

#define LOG(myRobot, stateName, state) {DO_PERIODIC(25, Logger::LogAction(myRobot, __FILE__, __LINE__, stateName, state))}

class Logger {
public:
	static void LogState(RobotModel* myRobot, RemoteControl *myHumanControl);
	static void LogAction(RobotModel* myRobot, const std::string& fileName, int line,
			const std::string& stateName, bool state);
	static std::string GetTimeStamp(const char* fileName);
private:
	static std::ofstream logData, logAction;
};

#endif /* SRC_LOGGER_H_ */
