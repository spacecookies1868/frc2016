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
#include "nivision.h"
#include "UltrasonicSensor.h"

// todo cache current and voltage methods so theyre called only once per loop

class RobotModel {
public:
	enum Wheels {kLeftWheels, kRightWheels, kAllWheels};

	RobotModel();
	~RobotModel() {}

	void SetWheelSpeed(Wheels w, double speed);
	float GetWheelSpeed(Wheels w);

	void InitServo(double angle);
	void SetServo(double startAngle, double endAngle, double deltaAngle);
	double GetServoAngle();
	bool GetServoDirection();

	double GetNavXYaw();
	double GetNavXRoll();
	double GetNavXPitch();
	void ZeroNavXYaw();

	void Reset();

	void UpdateCurrent();

	double GetVoltage();
	double GetCurrent(int channel);
	double GetCompressorCurrent();
	double GetRIOCurrent();
	double GetTotalEnergy();
	double GetTotalCurrent();
	double GetTotalPower();

	double GetLeftEncoderVal();
	double GetRightEncoderVal();
	void ResetDriveEncoders();

	double GetPressureSensorVal();

	double GetUltrasonicDistance();

	void RefreshIni();
	void ResetTimer();

	bool IsLowGear();
	void ShiftToLowGear();
	void ShiftToHighGear();

	double GetTime();

	//Superstructure accessor and mutator methods for RobotModel
	bool IsIntakeArmDown();
	void MoveIntakeArmUp();
	void MoveIntakeArmDown();
	void ChangeIntakeArmState();

	double GetIntakeMotorSpeed();
	void SetIntakeMotorSpeed(double speed);

	bool IsDefenseManipDown();
	void MoveDefenseManipUp();
	void MoveDefenseManipDown();
	void ChangeDefenseManipState();

	double GetOuttakeMotorSpeed();
	void SetOuttakeMotorSpeed(double speed);

	double GetOuttakeEncoderVal();
	void ResetOuttakeEncoders();

	void SetCompressorStop();

	Image* GetCameraImage();

	Ini* pini;
	TableReader* gripLines;
private:
	bool isLowGear;
	Compressor *compressor;

	PowerDistributionPanel* pdp;
	double leftDriveACurrent, leftDriveBCurrent, rightDriveACurrent, rightDriveBCurrent,
		roboRIOCurrent, compressorCurrent, intakeCurrent;

	Timer *timer;

	//Actuators
	Victor *leftDriveMotorA, *leftDriveMotorB, *rightDriveMotorA, *rightDriveMotorB, *intakeMotor, *outtakeMotorA, *outtakeMotorB;
	Servo* servo;
	double servoAngle;
	bool servoDirection;

	// Solenoids
	Solenoid *gearShiftSolenoid, *intakeArmSolenoidA, *intakeArmSolenoidB, *defenseManipSolenoidA, *defenseManipSolenoidB;

	//Sensors
	Encoder *leftEncoder, *rightEncoder;
	AnalogInput *pressureSensor;
	UltrasonicSensor *ultra;

#if USE_CAMERA
//	AxisCamera *camera;
//	Image *frame;
#endif

#if USE_NAVX
	//Port the NavX plugs into
	AHRS *navx;
#endif
};

#endif
