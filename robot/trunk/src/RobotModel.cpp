 #include "RobotModel.h"

#define PI 3.141592653589

#include "Debugging.h"

RobotModel::RobotModel() {
	pdp = new PowerDistributionPanel();

	leftDriveMotorA = new Victor(LEFT_DRIVE_MOTOR_A_PWM_PORT);
	leftDriveMotorB = new Victor(LEFT_DRIVE_MOTOR_B_PWM_PORT);
	rightDriveMotorA = new Victor(RIGHT_DRIVE_MOTOR_A_PWM_PORT);
	rightDriveMotorB = new Victor(RIGHT_DRIVE_MOTOR_B_PWM_PORT);
	intakeMotor = new Victor(INTAKE_MOTOR_PWM_PORT);
	outtakeMotorA = new Victor(OUTTAKE_MOTOR_A_PWM_PORT);
	outtakeMotorB = new Victor(OUTTAKE_MOTOR_B_PWM_PORT);

	leftDriveMotorA->SetSafetyEnabled(false);
	leftDriveMotorB->SetSafetyEnabled(false);
	rightDriveMotorA->SetSafetyEnabled(false);
	rightDriveMotorB->SetSafetyEnabled(false);
	intakeMotor->SetSafetyEnabled(false);
	outtakeMotorA->SetSafetyEnabled(false);
	outtakeMotorB->SetSafetyEnabled(false);

	isLowGear = false;

	servo = new Servo(SERVO_MOTOR_PORT);
	servoAngle = servo->GetAngle();
	servoDirection = true;

	gearShiftSolenoid = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, GEAR_SHIFT_SOLENOID_PORT);
	intakeArmSolenoidA = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, INTAKE_SOLENOID_A_PORT);
	intakeArmSolenoidB = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, INTAKE_SOLENOID_B_PORT);
	defenseManipSolenoidA = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, DEFENSE_MANIP_SOLENOID_A_PORT);
	defenseManipSolenoidB = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, DEFENSE_MANIP_SOLENOID_B_PORT);

	leftEncoder = new Encoder(LEFT_ENCODER_A_PWM_PORT, LEFT_ENCODER_B_PWM_PORT, true);
	rightEncoder = new Encoder(RIGHT_ENCODER_A_PWM_PORT, RIGHT_ENCODER_B_PWM_PORT, true);

	pressureSensor = new AnalogInput(PRESSURE_SENSOR_PORT);
	pressureSensor->SetAverageBits(2);

	ultra = new UltrasonicSensor(ULTRASONIC_SENSOR_PORT);

	leftDriveACurrent = 0;
	leftDriveBCurrent = 0;
	rightDriveACurrent = 0;
	rightDriveBCurrent = 0;
	roboRIOCurrent = 0;
	compressorCurrent = 0;
	intakeCurrent = 0;

//	// 8 inch wheels (2/3 ft), 256 tics per rotation
	leftEncoder->SetDistancePerPulse(((2.0/3.0) * PI) / 256.0);
	rightEncoder->SetDistancePerPulse(((2.0/3.0) * PI) / 256.0);
//
	/*
	 * SKETCH VALUES
	 */
//	leftEncoder->SetDistancePerPulse(0.0104);
//	rightEncoder->SetDistancePerPulse(0.0104);
#if USE_CAMERA
	camera = new AxisCamera("10.18.68.11");
	frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0); // 0 bordersize
#endif

	compressor = new Compressor(PNEUMATICS_CONTROL_MODULE_ID);
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
	SetWheelSpeed(RobotModel::kAllWheels, 0.0);
}

void RobotModel::SetWheelSpeed(Wheels w, double speed) {
	//IMPORTANT: Check which motors need to be inverted.

	switch (w) {
	case (kLeftWheels):
/* comp bot inverted
		leftDriveMotorA->Set(speed);
		leftDriveMotorB->Set(speed);
	*/
		leftDriveMotorA->Set(speed);
		leftDriveMotorB->Set(speed);
		break;
	case (kRightWheels):
/* comp bot inverted
		rightDriveMotorA->Set(-speed);
		rightDriveMotorB->Set(-speed);
*/
		rightDriveMotorA->Set(-speed);
		rightDriveMotorB->Set(-speed);
		break;
	case (kAllWheels):
/* comp bot inverted
		leftDriveMotorA->Set(speed);
		leftDriveMotorB->Set(speed);
		rightDriveMotorA->Set(-speed);
		rightDriveMotorB->Set(-speed);
*/
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

void RobotModel::InitServo(double angle) {
	servo->SetAngle(angle);
}

void RobotModel::SetServo(double startAngle, double endAngle, double deltaAngle) {
	servoAngle = servo->GetAngle();
	if (servoAngle >= endAngle){
		servoDirection = false;
	} else if (servoAngle <= startAngle){
		servoDirection = true;
	}

	if (servoDirection == true){
		servoAngle += deltaAngle;
	} else {
		servoAngle -= deltaAngle;
	}
//	printf("Start angle: %f \n", startAngle);
//	printf("End Angle: %f \n", endAngle);
//	printf("Delta Angle: %f \n", deltaAngle);
//	printf("servoAnlge: %f \n", servoAngle);
	servo->SetAngle(servoAngle);
}

double RobotModel::GetServoAngle() {
	servoAngle = servo->GetAngle();
	return servoAngle;
}

bool RobotModel::GetServoDirection() {
	return servoDirection;
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

void RobotModel::UpdateCurrent() {
	leftDriveACurrent = pdp->GetCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN);
	leftDriveBCurrent = pdp->GetCurrent(LEFT_DRIVE_MOTOR_B_PDP_CHAN);
	rightDriveACurrent = pdp->GetCurrent(RIGHT_DRIVE_MOTOR_A_PDP_CHAN);
	rightDriveBCurrent = pdp->GetCurrent(RIGHT_DRIVE_MOTOR_B_PDP_CHAN);
	intakeCurrent = pdp->GetCurrent(RIGHT_DRIVE_MOTOR_B_PDP_CHAN);
	compressorCurrent = compressor->GetCompressorCurrent();
	roboRIOCurrent = ControllerPower::GetInputCurrent();
}

double RobotModel::GetVoltage() {
	return pdp->GetVoltage();
}

double RobotModel::GetTotalCurrent() {
	return pdp->GetTotalCurrent();
}

double RobotModel::GetTotalEnergy() {
	return pdp->GetTotalEnergy();
}

double RobotModel::GetTotalPower() {
	return pdp->GetTotalPower();
}

/**
 * @param channel PDP channels from 0 to 15
 * check robotports for channels
 */
double RobotModel::GetCurrent(int channel) {
	switch(channel) {
	case RIGHT_DRIVE_MOTOR_A_PDP_CHAN:
		return rightDriveACurrent;
		break;
	case RIGHT_DRIVE_MOTOR_B_PDP_CHAN:
		return rightDriveBCurrent;
		break;
	case LEFT_DRIVE_MOTOR_A_PDP_CHAN:
		return leftDriveACurrent;
		break;
	case LEFT_DRIVE_MOTOR_B_PDP_CHAN:
		return leftDriveBCurrent;
		break;
	case INTAKE_MOTOR_PDP_CHAN:
		return intakeCurrent;
		break;
	default:
		return -1;
	}
}

double RobotModel::GetCompressorCurrent() {
	return compressorCurrent;
}

double RobotModel::GetRIOCurrent() {
	return roboRIOCurrent;
}

void RobotModel::ResetTimer() {
	timer->Reset();
}

double RobotModel::GetTime() {
	return timer->Get();
}

double RobotModel::GetLeftEncoderVal() {
	// IF PRACTICE return -leftEncoder->GetDistance();
	//IF COMP
	//printf("Left 2 Stuff %i\n", leftEncoder->CheckDigitalChannel(2));
	//printf("Left 3 Stuff %i\n", leftEncoder->CheckDigitalChannel(3));
	//printf("Left raw %i\n", leftEncoder->GetRaw());
	return leftEncoder->GetDistance();
}

double RobotModel::GetRightEncoderVal() {
	/*
	 * inverted
	 */

	//IF PRACTICE return rightEncoder->GetDistance();
	//printf("Right 0 Stuff %i\n", rightEncoder->CheckDigitalChannel(0));
	//printf("Right 1 Stuff %i\n", rightEncoder->CheckDigitalChannel(1));
	//printf("Right raw %i\n", rightEncoder->GetRaw());
	return rightEncoder->GetDistance();
}

void RobotModel::ResetDriveEncoders() {
	leftEncoder->Reset();
	rightEncoder->Reset();
}

double RobotModel::GetPressureSensorVal() {
	return 250 * (pressureSensor->GetAverageVoltage() / 5) - 25;
}

double RobotModel::GetUltrasonicDistance() {
	return ultra->GetRangeInInches();
}

void RobotModel::RefreshIni() {
	delete pini;
	pini = new Ini("/home/lvuser/robot.ini");
}

bool RobotModel::IsIntakeArmDown() {
	return intakeArmSolenoidA->Get();
}

void RobotModel::MoveIntakeArmUp() {
	DO_PERIODIC(1, printf("Move intake arm up\n"));
	intakeArmSolenoidA->Set(false);
	intakeArmSolenoidB->Set(true);
}

void RobotModel::MoveIntakeArmDown() {
	DO_PERIODIC(1, printf("Move intake arm down\n"));
	intakeArmSolenoidA->Set(true);
	intakeArmSolenoidB->Set(false);
}

void RobotModel::ChangeIntakeArmState() {
	if (IsIntakeArmDown()) {
		MoveIntakeArmUp();
	} else {
		MoveIntakeArmDown();
	}
}

double RobotModel::GetIntakeMotorSpeed() {
	return intakeMotor->Get();
}

void RobotModel::SetIntakeMotorSpeed(double speed) {
	DO_PERIODIC(20, printf("Set intake speed to %f\n", speed));
	intakeMotor->Set(speed);
}

bool RobotModel::IsDefenseManipDown() {
	//TODO check that this is the correct solenoid
	return defenseManipSolenoidA->Get();
}

void RobotModel::MoveDefenseManipUp() {
	//TODO check that this is the correct direction
	DO_PERIODIC(1, printf("Move defense arm up\n"));
	defenseManipSolenoidA->Set(false);
	defenseManipSolenoidB->Set(true);
}

void RobotModel::MoveDefenseManipDown() {
	//TODO check that this is the correct direction
	DO_PERIODIC(1, printf("Move defense arm down\n"));
	defenseManipSolenoidA->Set(true);
	defenseManipSolenoidB->Set(false);
}

void RobotModel::ChangeDefenseManipState() {
	if (IsDefenseManipDown()) {
		MoveDefenseManipUp();
	} else {
		MoveDefenseManipDown();
	}
}

double RobotModel::GetOuttakeMotorSpeed() {
	return outtakeMotorA->Get();
}

void RobotModel::SetOuttakeMotorSpeed(double speed) {
	//TODO check that this is the correct direction
	DO_PERIODIC(20, printf("Set outtake speed to %f\n", speed));
	outtakeMotorA->Set(-speed);
	outtakeMotorB->Set(speed);
}

double RobotModel::GetOuttakeEncoderVal() {
	return 0.0;
}

void RobotModel::ResetOuttakeEncoders() {

}

void RobotModel::SetCompressorStop() {
	compressor->Stop();
}
#if USE_CAMERA
Image* RobotModel::GetCameraImage() {
	camera->GetImage(frame);
	return frame;
}
#endif
