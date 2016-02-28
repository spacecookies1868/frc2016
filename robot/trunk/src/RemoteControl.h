#ifndef REMOTECONTROL_H
#define REMOTECONTROL_H

class RemoteControl {
public:
	enum Joysticks {kLeftJoy, kRightJoy};
	enum Axes {kX, kY};

	virtual void ReadControls() = 0;

	//Drive joystick accessors
	virtual double GetJoystickValue(Joysticks j, Axes a) = 0;

	//Drive controller button accessors
	virtual bool GetReverseDriveDesired() = 0;
	virtual bool GetLowGearDesired() = 0;
	virtual bool GetArcadeDriveDesired() = 0;
	virtual bool GetQuickTurnDesired() = 0;
	virtual uint32_t GetDefense() = 0;

	//Superstructure controller button accessors
	virtual bool GetDefenseManipDesired() = 0;
	virtual bool GetIntakePistonDesired() = 0;
	virtual bool GetIntakeMotorForwardDesired() = 0;
	virtual bool GetIntakeMotorReverseDesired() = 0;
	virtual bool GetOuttakeDesired() = 0;
	virtual bool GetManualOuttakeForwardDesired() = 0;
	virtual bool GetManualOuttakeReverseDesired() = 0;

	virtual ~RemoteControl() {}
};

#endif
