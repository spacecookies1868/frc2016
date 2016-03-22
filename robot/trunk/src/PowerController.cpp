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
	totalVoltage = 0;

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

	driveCurrentLimit = 0; // pdp measures amps inaccurately :. these are in chloeamps now
	intakeCurrentLimit = 0;
	totalCurrentLimit = 0;
	voltageFloor = 0;
	pressureFloor = 0;

	driveWeight = 0.8 * (totalVoltage / 13);
	compWeight = 0.5 * (totalVoltage / 13) * (pressureFloor / robot->GetPressureSensorVal());
	intakeWeight = 0.6 * (totalVoltage / 13) * (humanControl->GetIntakeMotorForwardDesired() * humanControl->GetIntakeMotorReverseDesired());
}

void PowerController::Update(double currTimeSec, double deltaTimeSec) {
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

	driveCurrentLimit *= (totalVoltage / 12);
	intakeCurrentLimit *= (totalVoltage / 12);

	driveWeight = 0.8 * (totalVoltage / 12);
	compWeight = 0.5 * (totalVoltage / 12) * (pressureFloor / robot->GetPressureSensorVal());
	intakeWeight = 0.6 * (totalVoltage / 12) *
		(humanControl->GetIntakeMotorForwardDesired() * humanControl->GetIntakeMotorReverseDesired());
	// todo add outtake motors
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
	// todo scale drivecurrentlimit as voltage decreases
	double diffCurr = avgLeftCurr - driveCurrentLimit;
	double scaledSpeed =  (robot->GetWheelSpeed(RobotModel::kLeftWheels)
		- (diffCurr + 4.84) / 47.4) * totalVoltage / 13;
	if (diffCurr >= 0) {
		LOG(robot, "l drive maxed", 1);
		robot->SetWheelSpeed(RobotModel::kLeftWheels, scaledSpeed);
	}

	diffCurr = avgRightCurr - driveCurrentLimit;
	scaledSpeed =  (robot->GetWheelSpeed(RobotModel::kRightWheels)
		- (diffCurr + 4.84) / 47.4) * totalVoltage / 13;
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

double PowerController::GetAverage(std::vector<double> v) {
	double avg = 0;
	for (size_t i = 1; i < (v.size() + 1); i++) {
		avg += (v[i] - avg) / i;
	}
	return avg;
}

void PowerController::RefreshIni() {
	driveCurrentLimit = robot->pini->getf("POWERBUDGET", "driveCurrentLimit", 40);
	intakeCurrentLimit = robot->pini->getf("POWERBUDGET", "intakeCurrentLimit", 20);
	totalCurrentLimit = robot->pini->getf("POWERBUDGET", "totalCurrentLimit", 160);
	voltageFloor = robot->pini->getf("POWERBUDGET", "voltageFloor", 7);
	pressureFloor = robot->pini->getf("POWERBUDGET", "pressureFloor", 60);
	size = robot->pini->getf("POWERBUDGET", "movingAverageSize", 5);
}

void PowerController::Reset() {
	totalCurrent = 0;
	totalVoltage = 0;
	avgLeftCurr = 0;
	avgRightCurr = 0;
	avgIntakeCurr = 0;
	avgCompCurr = 0;
	avgRioCurr = 0;
}

PowerController::~PowerController() {

}
