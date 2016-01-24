#ifndef REMOTECONTROL_H_
#define REMOTECONTROL_H_

class RemoteControl {
public:
	enum Joysticks {kLeftJoy, kRightJoy};
	enum Axes {kX, kY};

	virtual void ReadControls() = 0;

	virtual double GetJoystickValues(Joysticks j, Axes a) = 0;
	virtual bool GetArmControlButtonDown() = 0;
	virtual bool GetArmControlButtonUp() = 0;
	virtual bool GetIntakeButtonIn() = 0;
	virtual bool GetIntakeButtonOut() = 0;
	virtual bool GetArmControlButtonPressed() = 0;

	virtual ~RemoteControl() {}
};



#endif /* REMOTECONTROL_H_ */
