#ifndef REMOTECONTROL_H
#define REMOTECONTROL_H

//This file states all methods that are defined in ControlBoard
//ControlBoard inherits RemoteControl, which makes it easier to switch to new driver stations
class RemoteControl {
public:
	//Joysticks and Axes for switch case
	enum Joysticks {kLeftJoy, kRightJoy};
	enum Axes {kX, kY};

	virtual void ReadControls() = 0;

	//Drive joystick accessors
	virtual double GetJoystickValue(Joysticks j, Axes a) = 0;

	//Drive controller button accessors
	virtual bool GetReverseDriveDesired() = 0;
	virtual bool GetGearShiftDesired() = 0;
	virtual bool GetArcadeDriveDesired() = 0;
	virtual bool GetQuickTurnDesired() = 0;
	virtual bool GetPivotButtonDesired() = 0;
	virtual bool GetPivotSwitchDesired() = 0;
	virtual double GetDesiredAngle() = 0;

	//Auto buttons
	virtual uint32_t GetDefense() = 0;
	virtual uint32_t GetDefensePosition() = 0;
	virtual bool GetStopAutoDesired() = 0;

	//Superstructure controller button accessors
	virtual bool GetDefenseManipToggleDesired() = 0;
	virtual bool GetDefenseManipDownDesired() = 0;
	virtual bool GetIntakePistonToggleDesired() = 0;
	virtual bool GetIntakePistonDownDesired() = 0;
	virtual bool GetIntakeMotorForwardDesired() = 0;
	virtual bool GetIntakeMotorReverseDesired() = 0;
	virtual bool GetBallInIntakeDesired() = 0;
	virtual bool GetOuttakeDesired() = 0;
	virtual bool GetManualOuttakeForwardDesired() = 0;
	virtual bool GetManualOuttakeReverseDesired() = 0;
	virtual bool GetBrakeDesired() = 0;
	virtual bool GetJustBeforeDisableBrakeDesired() = 0;

	//Power controller button accessors
	virtual bool GetPowerBudgetDesired() = 0;
	virtual ~RemoteControl() {}
};

#endif
