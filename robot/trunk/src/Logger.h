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

#define LOG(myRobot, stateName, state) (Logger::LogAction(myRobot, __FILE__, __LINE__, stateName, state))

class Logger {
public:
	static void LogState(RobotModel* myRobot, RemoteControl *myHumanControl);
	static void LogAction(RobotModel* myRobot, const std::string& fileName, int line,
			const std::string& stateName, bool state);
private:
	static std::string dataFilePath, actionFilePath;
	static std::ofstream logData, logAction;
};

#endif /* SRC_LOGGER_H_ */
