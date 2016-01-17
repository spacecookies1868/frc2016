#ifndef REMOTECONTROL_H
#define REMOTECONTROL_H

class RemoteController {
public:
	enum Joysticks {kLeftJoy, kRightJoy};
	typedef enum Axes {kX, kY} uint32_t;

	virtual void ReadControls() = 0;

	virtual double GetJoystickValue(Joysticks j, Axes a) = 0;

	virtual bool ReverseDriveDesired() = 0;

	virtual ~RemoteController() {}
};

#endif
