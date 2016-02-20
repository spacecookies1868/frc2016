#include <RobotModel.h>

RobotModel::RobotModel() {
	navx = new AHRS(SPI::Port::kMXP);
	navx->ZeroYaw();

	leftA = new Talon(1);
	leftB = new Talon(0);
	rightA = new Talon(2);
	rightB = new Talon(5);

}

void RobotModel::SetWheelSpeed(Wheels w, double speed) {
	switch (w) {
	case (kLeftWheels):
		leftA->Set(speed);
		leftB->Set(speed);
		break;
	case (kRightWheels):
		rightA->Set(-speed);
		rightB->Set(-speed);
		break;
	case (kAllWheels):
		leftA->Set(speed);
		leftB->Set(speed);
		rightA->Set(-speed);
		rightB->Set(-speed);
		break;
	}
}

float RobotModel::GetYaw() {
	return navx->GetYaw();
}

float RobotModel::GetPitch() {
	return navx->GetPitch();
}

float RobotModel::GetRoll() {
	return navx->GetRoll();
}

void RobotModel::ZeroYaw() {
	navx->ZeroYaw();
}

RobotModel::~RobotModel() {

}

