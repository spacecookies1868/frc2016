#include "RobotModel.h"

#define PI 3.141592653589

#include "Debugging.h"

RobotModel::RobotModel() {
	pdp = new PowerDistributionPanel();

	leftDriveMotorA = new Victor(LEFT_DRIVE_MOTOR_A_PWM_PORT);
	leftDriveMotorB = new Victor(LEFT_DRIVE_MOTOR_B_PWM_PORT);
	rightDriveMotorA = new Victor(RIGHT_DRIVE_MOTOR_A_PWM_PORT);
	rightDriveMotorB = new Victor(RIGHT_DRIVE_MOTOR_B_PWM_PORT);

	leftDriveMotorA->SetExpiration(0.5);
	leftDriveMotorB->SetExpiration(0.5);
	rightDriveMotorA->SetExpiration(0.5);
	rightDriveMotorB->SetExpiration(0.5);

	leftDriveMotorA->SetSafetyEnabled(false);
	leftDriveMotorB->SetSafetyEnabled(false);
	rightDriveMotorA->SetSafetyEnabled(false);
	rightDriveMotorB->SetSafetyEnabled(false);

	isLowGear = false;

	gearShiftSolenoid = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, GEAR_SHIFT_SOLENOID_PORT);


	leftEncoder = new Encoder(LEFT_ENCODER_A_PWM_PORT, LEFT_ENCODER_B_PWM_PORT, true);
	rightEncoder = new Encoder(RIGHT_ENCODER_A_PWM_PORT, RIGHT_ENCODER_B_PWM_PORT, true);

//	// 8 inch wheels (2/3 ft), 256 tics per rotation
	leftEncoder->SetDistancePerPulse(((2.0/3.0) * PI) / 256.0);
	rightEncoder->SetDistancePerPulse(((2.0/3.0) * PI) / 256.0);
//
	/*
	 * SKETCH VALUES
	 */
//	leftEncoder->SetDistancePerPulse(0.0104);
//	rightEncoder->SetDistancePerPulse(0.0104);

	compressor = new Compressor(COMPRESSOR_PORT);
#if USE_NAVX
	//serialPort = new SerialPort(57600, SerialPort::kMXP);
	navx = new AHRS(SPI::Port::kMXP);
#endif

	timer = new Timer();
	timer->Start();

	pini = new Ini("/home/lvuser/robot.ini");
	gripLines = new TableReader("GRIP/myLinesReport");
}

void RobotModel::Reset() {
	isLowGear = !gearShiftSolenoid->Get();
	gripLines->Reset();
	ResetDriveEncoders();
	ResetOuttakeEncoders();
}

void RobotModel::SetWheelSpeed(Wheels w, double speed) {
	//IMPORTANT: Check which motors need to be inverted.

	switch (w) {
	case (kLeftWheels):
		leftDriveMotorA->Set(speed);
		leftDriveMotorB->Set(speed);
		break;
	case (kRightWheels):
		rightDriveMotorA->Set(-speed);
		rightDriveMotorB->Set(-speed);
		break;
	case (kAllWheels):
		leftDriveMotorA->Set(speed);
		leftDriveMotorB->Set(speed);
		rightDriveMotorA->Set(-speed);
		rightDriveMotorB->Set(-speed);
		break;
	}
}

float RobotModel::GetWheelSpeed(Wheels w) {
	//IMPORTANT: Check which motors need to be inverted.
	switch(w) {
	case (kLeftWheels):
		return leftDriveMotorA->Get();
		break;
	case (kRightWheels):
		return rightDriveMotorA->Get();
		break;
	default:
		return 0.0;
		break;
	}
}

double RobotModel::GetNavXYaw() {
#if USE_NAVX
	return navx->GetYaw();
#else
	return 0.0;
#endif
}

double RobotModel::GetNavXRoll() {
#if USE_NAVX
	return navx->GetRoll();
#else
	return 0.0;
#endif
}

double RobotModel::GetNavXPitch() {
#if USE_NAVX
	return navx->GetPitch();
#else
	return 0.0;
#endif
}

void RobotModel::ZeroNavXYaw() {
#if USE_NAVX
	navx->ZeroYaw();
#endif
}

bool RobotModel::IsLowGear() {
	return isLowGear;
}

void RobotModel::ShiftToHighGear() {
	gearShiftSolenoid->Set(false);
	isLowGear = false;
}

void RobotModel::ShiftToLowGear() {
	gearShiftSolenoid->Set(true);
	isLowGear = true;
}

double RobotModel::GetVoltage() {
	return pdp->GetVoltage();
}

void RobotModel::ResetTimer() {
	timer->Reset();
}

double RobotModel::GetTime() {
	return timer->Get();
}

double RobotModel::GetLeftEncoderVal() {
	return -leftEncoder->GetDistance();
}

double RobotModel::GetRightEncoderVal() {
	/*
	 * inverted
	 */
	return rightEncoder->GetDistance();
}

void RobotModel::ResetDriveEncoders() {
	leftEncoder->Reset();
	rightEncoder->Reset();
}

void RobotModel::RefreshIni() {
	delete pini;
	pini = new Ini("/home/lvuser/robot.ini");
}

//TODO Fill in all of these methods
bool RobotModel::IsIntakeArmDown() {
	return false;
}

void RobotModel::MoveIntakeArmUp() {
	DO_PERIODIC(1, printf("Move intake arm up\n"));
}

void RobotModel::MoveIntakeArmDown() {
	DO_PERIODIC(1, printf("Move intake arm down\n"));
}

void RobotModel::ChangeIntakeArmState() {
	if (IsIntakeArmDown()) {
		MoveIntakeArmUp();
	} else {
		MoveIntakeArmDown();
	}
}

double RobotModel::GetIntakeMotorSpeed() {
	return 0.0;
}

void RobotModel::SetIntakeMotorSpeed(double speed) {
	DO_PERIODIC(20, printf("Set intake speed to %f\n", speed));
}

bool RobotModel::IsDefenseManipDown() {
	return false;
}

void RobotModel::MoveDefenseManipUp() {
	DO_PERIODIC(1, printf("Move defense arm up\n"));
}

void RobotModel::MoveDefenseManipDown() {
	DO_PERIODIC(1, printf("Move defense arm down\n"));
}

void RobotModel::ChangeDefenseManipState() {
	if (IsDefenseManipDown()) {
		MoveDefenseManipUp();
	} else {
		MoveDefenseManipDown();
	}
}

double RobotModel::GetOuttakeMotorSpeed() {
	return 0.0;
}

void RobotModel::SetOuttakeMotorSpeed(double speed) {
	DO_PERIODIC(1, printf("Set outtake speed to %f\n", speed));
}

double RobotModel::GetOuttakeEncoderVal() {
	return 0.0;
}

void RobotModel::ResetOuttakeEncoders() {

}

