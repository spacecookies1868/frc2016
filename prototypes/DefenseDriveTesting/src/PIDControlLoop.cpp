#include "PIDControlLoop.h"
#include <math.h>

PIDConfig::PIDConfig() {
	pFac = 0.0;
	iFac = 0.0;
	dFac = 0.0;
	maxAbsOutput = 0.0;
	maxAbsError = 0.0;
	maxAbsDiffError = 0.0;
	desiredAccuracy = 0.0;
	maxAbsITerm = 0.1;
	minAbsError = 0.0;
	timeLimit = 1.5; // time threshold
}

PIDControlLoop::PIDControlLoop(PIDConfig* myConfig) {
	pidConfig = myConfig;
	Init(0.0, 0.0);
}

void PIDControlLoop::Init(double myInitialSensorValue,
						  double myDesiredSensorValue) {
	initialSensorValue = myInitialSensorValue;
	desiredSensorValue = myDesiredSensorValue;
	oldError = 0.0;
	sumError = 0.0;
	timeCount = 0.0; // for time threshold
}

void PIDControlLoop::Init(PIDConfig* myConfig, double myInitialSensorValue,
						  double myDesiredSensorValue) {
	pidConfig = myConfig;
	Init(myInitialSensorValue, myDesiredSensorValue);
	timeCount = 0.0; // for time threshold
}

// Returns the actuator value (motor speed, etc.)
double PIDControlLoop::Update(double currentSensorValue) {
	//printf("Current Sensor Value: %f\n", currentSensorValue);
	double error = desiredSensorValue - currentSensorValue;
	error = Saturate(error, pidConfig->maxAbsError);
	double diffError = 0.0;
	if (oldError != 0.0) {
		diffError = error - oldError;
		diffError = Saturate(diffError, pidConfig->maxAbsDiffError);
	}
	sumError += error;
	if (pidConfig->iFac > 0.0) {
		sumError = Saturate(sumError, (pidConfig->maxAbsITerm / pidConfig->iFac));
	}
	//printf("Error: %f, DiffError: %f, SumErrorL %f\n", error, diffError, sumError);
	double pTerm = pidConfig->pFac * error;
	double iTerm = pidConfig->iFac * sumError;
	double dTerm = pidConfig->dFac * diffError;
	double output = pTerm + iTerm + dTerm; //  PID
	output = Saturate(output, pidConfig->maxAbsOutput);
	//printf("PTerm: %f, ITerm: %f, DTerm: %f, Output: %f\n", pTerm, iTerm, dTerm, output);
	oldError = error;
	return output;
}

double PIDControlLoop::Update(double currValue, double desiredValue) {
	double error = desiredValue - currValue;
	error = Saturate(error, pidConfig->maxAbsError);
	double diffError = 0.0;
	if (oldError != 0.0) {
		diffError = error - oldError;
		diffError = Saturate(diffError, pidConfig->maxAbsDiffError);
	}
	sumError += error;
	if (pidConfig->iFac > 0.0) {
		sumError = Saturate(sumError,
				(pidConfig->maxAbsITerm / pidConfig->iFac));
	}
	//printf("Error: %f, DiffError: %f, SumErrorL %f\n", error, diffError, sumError);
	//printf("Pfac: %f, IFac: %f, DFac: %f\n", pidConfig->pFac, pidConfig->iFac, pidConfig->dFac);
	double pTerm = pidConfig->pFac * error;
	double iTerm = pidConfig->iFac * sumError;
	double dTerm = pidConfig->dFac * diffError;
	double output = pTerm + iTerm + dTerm; //  PID
	output = Saturate(output, pidConfig->maxAbsOutput);
	//printf("PTerm: %f, ITerm: %f, DTerm: %f, Output: %f\n", pTerm, iTerm, dTerm, output);
	if (Abs(output) < pidConfig->minAbsError) {
		output = 0.0;
	}
	oldError = error;
	return output;
}

double PIDControlLoop::Saturate(double value, double maxAbsValue) {
	//limits the value to maxAbsValue
	if (maxAbsValue > 0.0) {
		if (value > 0.0) {
			return fmin(value, maxAbsValue);
		} else {
			return fmax(value, -maxAbsValue);
		}
	} else {
		return value;
	}
}

bool PIDControlLoop::ControlLoopDone(double currentSensorValue) {
	//when the value returned is close enough to what we want
	return ControlLoopDone(currentSensorValue, 0.02);
}

/** when the value returned is close enough to what we want for a certain length of time.
	new and better version of ControlLoopDone(double currentSensorValue)
	@param currentSensorValue the current value of the sensor
	@param deltaTime the increment of time between each loop of the code
*/
bool PIDControlLoop::ControlLoopDone(double currentSensorValue, double deltaTime) {
	if (fabs(desiredSensorValue - currentSensorValue) <= pidConfig->desiredAccuracy) {
		// we want to start incrementing timeCount only if we're within the
		// threshold that we've set
		timeCount += deltaTime;

		if (timeCount >= pidConfig->timeLimit) {
			// we are done with the control loop once the counter has reached
			// the time threshold we set
			return true;
		} else {
			return false;
		}
	} else {
		// if the sensor value exits the range of "okay" desired values, then
		// reset the counter and start over
		timeCount = 0;
		return false;
	}
}

PIDConfig* PIDControlLoop::GetPIDConfig() {
	return pidConfig;
}

double PIDControlLoop::GetError() {
	return error;
}

double PIDControlLoop::GetDiffError() {
	return diffError;
}

double PIDControlLoop::GetSumError() {
	return sumError;
}

double PIDControlLoop::GetPTerm() {
	return pTerm;
}

double PIDControlLoop::GetITerm() {
	return iTerm;
}

double PIDControlLoop::GetDTerm() {
	return dTerm;
}

void PIDControlLoop::PrintPIDValues(const std::string& pidName) {
	//SmartDashboard::PutNumber(pidName + " error", error);
	//SmartDashboard::PutNumber(pidName + " diffError", diffError);
	//SmartDashboard::PutNumber(pidName + " summError", sumError);
	//SmartDashboard::PutNumber(pidName + " pTerm", pTerm);
	//SmartDashboard::PutNumber(pidName + " iTerm", iTerm);
}

double PIDControlLoop::Abs(double x) {
	if (x > 0.0) {
		return x;
	} else {
		return -x;
	}
}
