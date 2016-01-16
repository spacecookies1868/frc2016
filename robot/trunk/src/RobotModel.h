#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include "WPILib.h"
#include "RobotPorts2016.h"
#include "ini.h"
#include "Debugging.h"
#include <iostream>
#include <fstream>

class RobotModel {
public:
	//enum Wheels {kFrontLeftWheel, kRearLeftWheel, kFrontRightWheel, kRearRightWheel, kAllWheels};

	RobotModel();
	~RobotModel() {}
/*
	void SetWheelSpeed(Wheels w, double speed);
	float GetWheelSpeed(Wheels w);
*/
	double GetVoltage();
/*
	void EnableCompressor();
	void DisableCompressor();
	void ResetCompressor();
	bool GetCompressorState();
*/
	/*
	double GetFrontLeftEncoderVal();
	double GetFrontRightEncoderVal();
	double GetRearLeftEncoderVal();
	double GetRearRightEncoderVal();
	void ResetDriveEncoders();
	*/

	//void RefreshIniFile();
	void ResetTimer();

	Timer *timer;
	//Ini* pini;
	PowerDistributionPanel* pdp;

private:
	//Compressor *compressor;

	//Actuators
	//Victor *frontLeft, *rearLeft, *frontRight, *rearRight;

	//Sensors
//	Encoder *frontLeftEncoder, *rearLeftEncoder, *frontRightEncoder, *rearRightEncoder;
	SerialPort *serialPort;
};

#endif
