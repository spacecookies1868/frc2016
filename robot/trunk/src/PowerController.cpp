/*
 * PowerController.cpp
 *
 *  Created on: Feb 25, 2016
 *      Author: cmo
 */
#include "PowerController.h"

PowerController::PowerController(RobotModel* myRobot, RemoteControl* myHumanControl) {
	robot = myRobot;
	humanControl = myHumanControl;
	totalCurrent = 0;
	oldCurrent = 0;
	totalVoltage = robot->GetVoltage();
	oldVoltage = 0;
	diffVoltage = 0;

	avgLeftCurr = 0;
	avgRightCurr = 0;
	avgIntakeCurr = 0;
	avgCompCurr = 0;
	avgRioCurr = 0;

	size = 5;

	std::vector<double> pastLeftCurr(size, 0);
	std::vector<double> pastRightCurr(size, 0);
	std::vector<double> pastIntakeCurr(size, 0);
	std::vector<double> pastCompCurr(size, 0);
	std::vector<double> pastRioCurr(size, 0);

	driveCurrentLimit = 40; // in amps
	intakeCurrentLimit = 20;
	totalCurrentLimit = 160;
	voltageFloor = 7; // technically 6.8V but 7 to be safe
	pressureFloor = 60; // todo test!!!!!!!

	// todo put all these suckers in the ini?????
	driveWeight = 0.8 * (totalVoltage / 12);
	compWeight = 0.5 * (totalVoltage / 12) * (pressureFloor / robot->GetPressureSensorVal());
	intakeWeight = 0.6 * (totalVoltage / 12) * (humanControl->GetIntakeMotorForwardDesired() * humanControl->GetIntakeMotorReverseDesired());
}

void PowerController::Update(double currTimeSec, double deltaTimeSec) {
	oldCurrent = totalCurrent;
	oldVoltage = totalVoltage;

	pastLeftCurr.insert(pastLeftCurr.begin(),
			(robot->GetCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN) +
			robot->GetCurrent(LEFT_DRIVE_MOTOR_B_PDP_CHAN)) / 2);
	pastRightCurr.insert(pastRightCurr.begin(),
			(robot->GetCurrent(RIGHT_DRIVE_MOTOR_A_PDP_CHAN) +
			robot->GetCurrent(RIGHT_DRIVE_MOTOR_B_PDP_CHAN)) / 2);
	pastIntakeCurr.insert(pastIntakeCurr.begin(), robot->GetCurrent(INTAKE_MOTOR_PDP_CHAN));
	pastCompCurr.insert(pastCompCurr.begin(), robot->GetCompressorCurrent());
	pastRioCurr.insert(pastRioCurr.begin(), robot->GetRIOCurrent());

	pastLeftCurr.pop_back();
	pastRightCurr.pop_back();
	pastIntakeCurr.pop_back();
	pastCompCurr.pop_back();
	pastRioCurr.pop_back();

	avgLeftCurr = GetAverage(pastLeftCurr);
	avgRightCurr = GetAverage(pastRightCurr);
	avgIntakeCurr = GetAverage(pastIntakeCurr);
	avgCompCurr = GetAverage(pastCompCurr);
	avgRioCurr = GetAverage(pastRioCurr);

	totalCurrent = avgCompCurr + avgLeftCurr + avgRightCurr + avgIntakeCurr
			+ avgRioCurr;
	totalVoltage = robot->GetVoltage();

	driveWeight = 0.8 * (totalVoltage / 12);
	compWeight = 0.5 * (totalVoltage / 12) * (pressureFloor / robot->GetPressureSensorVal());
	intakeWeight = 0.6 * (totalVoltage / 12) *
		(humanControl->GetIntakeMotorForwardDesired() * humanControl->GetIntakeMotorReverseDesired());

	if (IsBatteryLow()) {
		LOG(robot, "battery low", true);
	}

	if (humanControl->GetPowerBudgetDesired()) {
		if (totalVoltage < (voltageFloor + 2) ) {
			LimitSingle();
			PriorityScale();
		}
	}
}

bool PowerController::IsBatteryLow() {
	if ((fabs(robot->GetVoltage() - voltageFloor) < 1) && totalCurrent < 20) {
		// todo 20 is an arbitrary value, test
		LOG(robot, "battery sucks", 1);
		return true;
	} else {
		return false;
	}
}

void PowerController::LimitSingle() {
	// linear regression model of current vs speed:
	// current = 47.4 * speed - 4.84 changes depending on BATTERY VOLTAGE!!!!!axqwaz
	double diffCurr = avgLeftCurr - driveCurrentLimit;
	double scaledSpeed =  robot->GetWheelSpeed(RobotModel::kLeftWheels)
		- (diffCurr + 4.84) / 47.4;
	if (diffCurr >= 0) {
		LOG(robot, "l drive maxed", 1);
		robot->SetWheelSpeed(RobotModel::kLeftWheels, scaledSpeed);
	}

	diffCurr = avgRightCurr - driveCurrentLimit;
	scaledSpeed =  robot->GetWheelSpeed(RobotModel::kRightWheels)
		- (diffCurr + 4.84) / 47.4;
	if (diffCurr >= 0) {
		LOG(robot, "r drive maxed", 1);
		robot->SetWheelSpeed(RobotModel::kRightWheels, scaledSpeed);
	}

	diffCurr = avgIntakeCurr - intakeCurrentLimit;
	// scaledSpeed = robot->GetIntakeMotorSpeed() -
	// todo add scaling for intake motor and outtake motor, compressor, roborio
}
void PowerController::PriorityScale() {
	double roboRIORatio = avgRioCurr / totalCurrent;

	double diffRatio = totalCurrent / totalCurrentLimit;
/*
	double leftDriveRatio = avgLeftCurr / totalCurrent;
	double rightDriveRatio = avgRightCurr / totalCurrent;
	double compressorRatio = avgCompCurr / totalCurrent;
	double intakeRatio = avgIntakeCurr / totalCurrent
	*/
	if (diffRatio >= 1) {
// todo test total current values
		LOG(robot, "current maxed", 1);
		double adjLeftWheelSpeed =  ((1 / diffRatio) - roboRIORatio) * robot->GetWheelSpeed(RobotModel::kLeftWheels);
		double adjRightWheelSpeed =  ((1 / diffRatio) - roboRIORatio) * robot->GetWheelSpeed(RobotModel::kRightWheels);
		robot->SetWheelSpeed(RobotModel::kLeftWheels, driveWeight * adjLeftWheelSpeed);
		robot->SetWheelSpeed(RobotModel::kRightWheels, driveWeight * adjRightWheelSpeed);
		// todo 0.7 is completely arbitrary -- test

		double adjIntakeSpeed =  ((1 / diffRatio) - roboRIORatio) * robot->GetIntakeMotorSpeed();
		robot->SetIntakeMotorSpeed(intakeWeight * adjIntakeSpeed);
		CompressorCut(); // todo incorporate compressor weight
	}
}

void PowerController::CompressorCut() {
	// double compressorRatio = avgCompCurr / totalCurrent;
	if ((totalCurrent > totalCurrentLimit) &&
			(robot->GetPressureSensorVal() > pressureFloor)) {
		robot->SetCompressorStop();
	}
}

void PowerController::Reset() {

}

double PowerController::GetAverage(std::vector<double> v) {
	double avg = 0;
	for (size_t i = 1; i < (v.size() + 1); i++) {
		avg += (v[i] - avg) / i;
	}
	return avg;
}

PowerController::~PowerController() {

}
