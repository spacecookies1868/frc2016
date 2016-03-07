/*
 * PowerController.h
 *
 *  Created on: Feb 25, 2016
 *      Author: cmo
 *
 *  priority:
 *  1. roboRIO -
 *  2. drive motors - 60%
 *  3. intake motors - 30%
 *  4. compressor - 10%
 *  - priority of compressor changes depending on amount of air
 *  - priority of intake changes depending if button is pressed
 *  - before priorityscaling, assign weights to everything
 *  - change priority scaling depending on voltage -> don't need to gradually go blind
 *  - increase voltage floor by 1-2V
 *  - take time into account by averaging past ~several amounts
 *
 *  rules:
 *  -switch on driver station to turn this whole thing off
 *  -if current is low and voltage is low, then the battery sucks
 *  -if individual motors > limit, scale them
 *  -if total current draw > limit, scale according to priority
 *  -if voltage < safe limit, scale current draw according to priority
 *	-if intake motors are drawing increasing current from
 *		0 (ie, operator wants to use them), and total limit is almost reached,
 *		scale drive motors more
 *	-if pressure is close to max todo: test how much air it takes to do things
 *		and current is close to max, cut compressor
 *
 */

#ifndef SRC_POWERCONTROLLER_H_
#define SRC_POWERCONTROLLER_H_

#include "RobotModel.h"
#include "ControlBoard.h"
#include "RemoteControl.h"
#include "Debugging.h"
#include "Logger.h"
#include <vector>

class PowerController {
	public:
		PowerController(RobotModel* myRobot, ControlBoard* myHumanControl);
		void Update(double currTimeSec, double deltaTimeSec);
		bool IsBatteryLow();
		void LimitSingle();
		void PriorityScale();
		void IntakeFocus();
		void CompressorCut();
		void RefreshIni();
		void Reset();

		virtual ~PowerController();
	private:
		RobotModel* robot;
		ControlBoard* humanControl;
		double totalCurrent, oldCurrent;

		double avgLeftCurr, avgRightCurr, avgIntakeCurr, avgCompCurr, avgRioCurr;
		int size; // number of past values to average
		std::vector<double> pastLeftCurr, pastRightCurr, pastIntakeCurr,
			pastCompCurr, pastRioCurr;

		double totalVoltage, oldVoltage, diffVoltage;

		double driveCurrentLimit, intakeCurrentLimit, totalCurrentLimit;
		double voltageFloor, pressureFloor;

		double driveWeight, compWeight, intakeWeight;

		double GetAverage(std::vector<double> v);
};

#endif /* SRC_POWERCONTROLLER_H_ */
