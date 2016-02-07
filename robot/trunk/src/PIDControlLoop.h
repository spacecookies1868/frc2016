#ifndef PIDCONTROLLOOP_H_
#define PIDCONTROLLOOP_H_

#include "RobotModel.h"
#include "WPILib.h"
#include <math.h>
#include <string.h>

struct PIDConfig {
 public:
	PIDConfig();
	double pFac;
	double iFac;
	double dFac;
	double maxAbsOutput;
	double maxAbsError;
	double maxAbsDiffError;
	double desiredAccuracy;
	double maxAbsITerm;
	double minAbsError;

	double timeLimit;  /*!< required time the sensor must be within the setpoint before ending control loop */
};

class PIDControlLoop {
 public:
	PIDControlLoop(PIDConfig* myConfig);
	~PIDControlLoop() {}
	void Init(double myInitialSensorValue, double desiredSensorValue);
	void Init(PIDConfig* myConfig, double myInitialSensorValue, double desiredSensorValue);
	double Update(double currentSensorValue); // Returns the actuator value (motor speed, etc.)
	double Update(double currValue, double desiredValue);
	bool ControlLoopDone(double currentSensorValue);
	bool ControlLoopDone(double currentSensorValue, double deltaTime);

	static double Saturate(double value, double maxAbsValue);

	PIDConfig* GetPIDConfig();

	double GetError();
	double GetDiffError();
	double GetSumError();
	double GetPTerm();
	double GetITerm();
	double GetDTerm();

	void PrintPIDValues(const std::string& pidName);

private:
	double Abs(double x);
	PIDConfig* pidConfig;
	double initialSensorValue;
	double desiredSensorValue;
	double oldError;
	double error;
	double sumError;
	double diffError;
	double pTerm;
	double iTerm;
	double dTerm;

	double timeCount;  /*!< counter to keep track of how long sensor value has been in range of setpoint */
};

#endif
