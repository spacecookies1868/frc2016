#include "AutoCommand.h"
#include <math.h>
#include "ini.h"
#include <iostream>
#include <string>

#define PI 3.14159265358979

WaitingCommand::WaitingCommand(double myWaitTimeSec) {
	waitTimeSec = myWaitTimeSec;
	timer = new Timer();
}

void WaitingCommand::Init() {
	timer->Start();
}

void WaitingCommand::Update(double currTimeSec, double deltaTimeSec) {
}

bool WaitingCommand::IsDone() {
	return (timer->Get() >= waitTimeSec);
}
