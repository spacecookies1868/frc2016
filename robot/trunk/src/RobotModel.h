#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include "WPILib.h"
#include "RobotPorts2016.h"
#include "ini.h"
#include "Debugging.h"
#include <iostream>
#include <fstream>
#include "navx/AHRS.h"
#include "TableReader.h"

class RobotModel {
public:
	enum Wheels {kLeftWheels, kRightWheels, kAllWheels};

	RobotModel();
	~RobotModel() {}

	void SetWheelSpeed(Wheels w, double speed);
	float GetWheelSpeed(Wheels w);

	double GetNavXYaw();
	double GetNavXRoll();
	double GetNavXPitch();
	void ZeroNavXYaw();

	void Reset();

	double GetVoltage();

	double GetLeftEncoderVal();
	double GetRightEncoderVal();
	void ResetDriveEncoders();

	void RefreshIni();
	void ResetTimer();

	bool IsLowGear();
	void ShiftToLowGear();
	void ShiftToHighGear();

	double GetTime();

	Ini* pini;
	TableReader* gripLines;
private:
	bool isLowGear;
	Compressor *compressor;

	PowerDistributionPanel* pdp;

	Timer *timer;

	//Actuators
	Victor *leftDriveMotorA, *leftDriveMotorB, *rightDriveMotorA, *rightDriveMotorB;

	// Solenoids
	Solenoid *gearShiftSolenoid;

	//Sensors
	Encoder *leftEncoder, *rightEncoder;


#if USE_NAVX
	//Port the NavX plugs into
	SerialPort *serialPort;
	AHRS *navx;
#endif
};

#endif
