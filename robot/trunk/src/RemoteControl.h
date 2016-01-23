#ifndef REMOTECONTROL_H
#define REMOTECONTROL_H

class RemoteControl {
public:
	enum Joysticks {kLeftJoy, kRightJoy};
	enum Axes {kX, kY};

	virtual void ReadControls() = 0;

	virtual double GetJoystickValue(Joysticks j, Axes a) = 0;

	virtual bool GetReverseDriveDesired() = 0;
	virtual bool GetLowGearDesired() = 0;

	virtual ~RemoteControl() {}
};

#endif
