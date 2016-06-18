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

class RobotModel {
public:
	enum Wheels {kLeftWheels, kRightWheels, kAllWheels};

	RobotModel();
	~RobotModel() {}

	void Reset(); //resets variables and objects

	void SetWheelSpeed(Wheels w, double speed); //sets the speed for a given wheel(s)
	float GetWheelSpeed(Wheels w); //returns the speed of a given wheel

	void InitServo(double angle); //sets the servo to an initial angle
	void SetServo(double startAngle, double endAngle, double deltaAngle); //sets the servo to the end angle at a speed proportional to the deltaAngle
	double GetServoAngle(); //returns the angle of the servo
	bool GetServoDirection(); //returns the direction the servo is turning

	double GetNavXYaw(); //returns the yaw
	double GetNavXRoll(); //returns the roll
	double GetNavXPitch(); //returns the pitch
	void ZeroNavXYaw(); //zeroes the initial yaw

	bool IsLowGear(); //returns if we are in low hear (or high gear)
	void ShiftToLowGear(); //shifts to low gear
	void ShiftToHighGear(); //shifts to high gear

	void UpdateCurrent(); //initializes variables pertaining to current

	double GetVoltage(); //returns the voltage
	double GetTotalEnergy(); //returns the total energy of the PDP
	double GetTotalCurrent(); //returns the total current of the PDP
	double GetTotalPower(); //returns the total power of the PDP
	double GetCurrent(int channel); //returns the current of a given channel
	double GetCompressorCurrent(); //returns the current of the compressor
	double GetRIOCurrent(); //returns the current of the roboRIO

	void ResetTimer(); //resets the timer
	double GetTime(); //returns the time

	double GetLeftEncoderVal(); //returns the distance of the left encoder
	double GetRightEncoderVal(); //returns the distance of the right encoder
	void ResetDriveEncoders(); //resets both the left and the right encoders

	double GetPressureSensorVal(); //returns the pressure

	double GetUltrasonicDistance(); //returns the distance of the ultrasonic sensor

	void RefreshIni(); //refreshes the ini file

	//Superstructure accessors and mutators in RobotModel
	bool IsIntakeArmDown(); //returns if the intake arm is down (or up)
	void MoveIntakeArmUp(); //moves intake arm up
	void MoveIntakeArmDown(); //moves intake arm down
	void ChangeIntakeArmState(); //changes the state of the intake arm (i.e. if up, move down)

	double GetIntakeMotorSpeed(); //returns the speed of the intake motor
	void SetIntakeMotorSpeed(double speed); //sets the speed of the intake motor
	bool GetIntakeSwitchState(); //returns if the intake switch is up (or down)

	bool IsDefenseManipDown(); //returns if the defense manipulator is down (or up)
	void MoveDefenseManipUp(); // moves the defense manipulator up
	void MoveDefenseManipDown(); // moves the defense manipulator down
	void ChangeDefenseManipState(); //changes the state of the defense manipulator (i.e. if up, move down)

	double GetOuttakeMotorSpeed(); //returns the speed of the outake motor
	void SetOuttakeMotorSpeed(double speed); //sets the speed of the outake motor
	double GetOuttakeEncoderVal(); //returns the distance of the outake motor encoder
	void ResetOuttakeEncoders(); //resets the outake motor encoder

	void SetCompressorStop(); //stops the compressor

	bool GetBrake(); //returns if the brake is on or off
	void SetBrakeOn(); //puts the break on
	void SetBrakeOff(); //takes the break off

	Image* GetCameraImage(); //returns the camera image
#if USE_USB_CAMERA
	USBCamera* usbCamera;
	Image* usbFrame;
#endif

	Ini* pini;
	TableReader* gripLines;
	Victor *leftDriveMotorA, *leftDriveMotorB, *rightDriveMotorA, *rightDriveMotorB, *intakeMotor, *outtakeMotorA, *outtakeMotorB;
private:
	bool isLowGear;
	Compressor *compressor;

	PowerDistributionPanel* pdp;
	double leftDriveACurrent, leftDriveBCurrent, rightDriveACurrent, rightDriveBCurrent,
		roboRIOCurrent, compressorCurrent, intakeCurrent;

	Timer *timer;

	//Actuators
	Servo* servo;
	double servoAngle;
	bool servoDirection;

	//Solenoids
	Solenoid *gearShiftSolenoid, *intakeArmSolenoidA, *intakeArmSolenoidB,
		*defenseManipSolenoidA, *defenseManipSolenoidB, *brakeSolenoidA, *brakeSolenoidB;

	//Sensors
	Encoder *leftEncoder, *rightEncoder, *outtakeEncoder1, *outtakeEncoder2;
	AnalogInput *pressureSensor;
	DigitalInput *intakeSwitch;
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
