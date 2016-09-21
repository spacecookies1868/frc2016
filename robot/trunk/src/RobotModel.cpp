 #include "RobotModel.h"

#define PI 3.141592653589

#include "Debugging.h"

// RobotModel constructor: initializes all variables and objects
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
	brakeSolenoidA = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, BRAKE_SOLENOID_A_PORT);
	brakeSolenoidB = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, BRAKE_SOLENOID_B_PORT);

	leftEncoder = new Encoder(LEFT_ENCODER_A_PWM_PORT, LEFT_ENCODER_B_PWM_PORT, true);
	rightEncoder = new Encoder(RIGHT_ENCODER_A_PWM_PORT, RIGHT_ENCODER_B_PWM_PORT, true);
	outtakeEncoder1 = new Encoder(OUTTAKE_ENCODER_1_A_PWM_PORT, OUTTAKE_ENCODER_1_B_PWM_PORT, true);
	outtakeEncoder2 = new Encoder(OUTTAKE_ENCODER_2_A_PWM_PORT, OUTTAKE_ENCODER_2_B_PWM_PORT, true);

	pressureSensor = new AnalogInput(PRESSURE_SENSOR_PORT);
	pressureSensor->SetAverageBits(2);

	intakeSwitch = new DigitalInput(INTAKE_SWITCH_PWM_PORT);

	ultra = new UltrasonicSensor(ULTRASONIC_SENSOR_PORT);

	leftDriveACurrent = 0;
	leftDriveBCurrent = 0;
	rightDriveACurrent = 0;
	rightDriveBCurrent = 0;
	roboRIOCurrent = 0;
	compressorCurrent = 0;
	intakeCurrent = 0;

	// 8 inch wheels (2/3 ft), 256 tics per rotation
	// changed to 7.5 inch wheels
	leftEncoder->SetDistancePerPulse(((7.4/12.0) * PI) / 256.0);
	rightEncoder->SetDistancePerPulse(((7.4/12.0) * PI) / 256.0);

#if USE_CAMERA
//	camera = new AxisCamera("10.18.68.11");
//	frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0); // 0 bordersize
#endif

#if USE_USB_CAMERA
	usbCamera = new USBCamera("cam0", true);
	usbCamera->OpenCamera();
	usbCamera->SetBrightness(50);
	usbCamera->SetWhiteBalanceManual(30);
//	usbCamera->SetExposureManual(30);
	usbCamera->SetExposureHoldCurrent();
//	usbCamera->SetExposureAuto();
//	usbCamera->SetWhiteBalanceAuto();
	usbCamera->UpdateSettings();
	usbCamera->StartCapture();
	usbFrame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
#endif

	compressor = new Compressor(PNEUMATICS_CONTROL_MODULE_ID);
#if USE_NAVX
	//serialPort = new SerialPort(57600, SerialPort::kMXP);
	navx = new AHRS(SPI::Port::kMXP);
#endif

	timer = new Timer();
	timer->Start();

	pini = new Ini("/home/lvuser/robot.ini");
	gripLines = new TableReader("GRIP/myLinesReport", "GRIP/myContReport");
}

//resets variables and objects
void RobotModel::Reset() {
	isLowGear = !gearShiftSolenoid->Get();
	gripLines->Reset();
	ResetDriveEncoders();
	ResetOuttakeEncoders();
	SetWheelSpeed(RobotModel::kAllWheels, 0.0);
}

//sets the speed for a given wheel(s)
void RobotModel::SetWheelSpeed(Wheels w, double speed) {
	switch (w) {
	case (kLeftWheels):
		leftDriveMotorA->Set(speed);
		leftDriveMotorB->Set(speed);
		break;
	case (kRightWheels):
		rightDriveMotorA->Set(-speed); //negative value since wheels are inverted on robot
		rightDriveMotorB->Set(-speed); //negative value since wheels are inverted on robot
		break;
	case (kAllWheels):
		leftDriveMotorA->Set(speed);
		leftDriveMotorB->Set(speed);
		rightDriveMotorA->Set(-speed); //negative value since wheels are inverted on robot
		rightDriveMotorB->Set(-speed); //negative value since wheels are inverted on robot
		break;
	}
}

//returns the speed of a given wheel
float RobotModel::GetWheelSpeed(Wheels w) {
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

//sets the servo to an initial angle
void RobotModel::InitServo(double angle) {
	servo->SetAngle(angle);
}

//sets the servo to the end angle at a speed proportional to the deltaAngle
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
	servo->SetAngle(servoAngle);
}

//returns the angle of the servo
double RobotModel::GetServoAngle() {
	servoAngle = servo->GetAngle();
	return servoAngle;
}

//returns the direction the servo is turning
bool RobotModel::GetServoDirection() {
	return servoDirection;
}

//returns the yaw
double RobotModel::GetNavXYaw() {
// if the navX is on the robot, return the value;
// otherwise, return a default value.
#if USE_NAVX
	return navx->GetYaw();
#else
	return 0.0;
#endif
}

//returns the roll
double RobotModel::GetNavXRoll() {
// if the navX is on the robot, return the value;
// otherwise, return a default value.
#if USE_NAVX
	return navx->GetRoll();
#else
	return 0.0;
#endif
}
//returns the pitch
double RobotModel::GetNavXPitch() {
// if the navX is on the robot, return the value;
// otherwise, return a default value.
#if USE_NAVX
	return navx->GetPitch();
#else
	return 0.0;
#endif
}
//zeroes the initial yaw
void RobotModel::ZeroNavXYaw() {
// if the navX is on the robot, zero the yaw.
#if USE_NAVX
	navx->ZeroYaw();
#endif
}

//returns if we are in low hear (or high gear)
bool RobotModel::IsLowGear() {
	return isLowGear;
}

//shifts to low gear
void RobotModel::ShiftToLowGear() {
	gearShiftSolenoid->Set(false);
	isLowGear = true;
}

//shifts to high gear
void RobotModel::ShiftToHighGear() {
	gearShiftSolenoid->Set(true);
	isLowGear = false;
}

//initializes variables pertaining to current
void RobotModel::UpdateCurrent() {
	leftDriveACurrent = pdp->GetCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN);
	leftDriveBCurrent = pdp->GetCurrent(LEFT_DRIVE_MOTOR_B_PDP_CHAN);
	rightDriveACurrent = pdp->GetCurrent(RIGHT_DRIVE_MOTOR_A_PDP_CHAN);
	rightDriveBCurrent = pdp->GetCurrent(RIGHT_DRIVE_MOTOR_B_PDP_CHAN);
	intakeCurrent = pdp->GetCurrent(RIGHT_DRIVE_MOTOR_B_PDP_CHAN);
	compressorCurrent = compressor->GetCompressorCurrent();
	roboRIOCurrent = ControllerPower::GetInputCurrent();
}

//returns the voltage
double RobotModel::GetVoltage() {
	return pdp->GetVoltage();
}

//returns the total energy of the PDP
double RobotModel::GetTotalCurrent() {
	return pdp->GetTotalCurrent();
}
//returns the total current of the PDP
double RobotModel::GetTotalEnergy() {
	return pdp->GetTotalEnergy();
}

//returns the total power of the PDP
double RobotModel::GetTotalPower() {
	return pdp->GetTotalPower();
}

//returns the current of a given channel
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

//returns the current of the compressor
double RobotModel::GetCompressorCurrent() {
	return compressorCurrent;
}

//returns the current of the roboRIO
double RobotModel::GetRIOCurrent() {
	return roboRIOCurrent;
}

//resets the time
void RobotModel::ResetTimer() {
	timer->Reset();
}

//returns the time
double RobotModel::GetTime() {
	return timer->Get();
}

//returns the distance of the left encoder
double RobotModel::GetLeftEncoderVal() {
	return leftEncoder->GetDistance();
}

//returns the distance of the right encoder
double RobotModel::GetRightEncoderVal() {
	return rightEncoder->GetDistance();
}

//resets both the left and the right encoders
void RobotModel::ResetDriveEncoders() {
	leftEncoder->Reset();
	rightEncoder->Reset();
}

//returns the pressure
double RobotModel::GetPressureSensorVal() {
	return 250 * (pressureSensor->GetAverageVoltage() / 5) - 25;
}

//returns the distance of the ultrasonic sensor
double RobotModel::GetUltrasonicDistance() {
	return ultra->GetRangeInInches();
}

//refreshes the ini file
void RobotModel::RefreshIni() {
	delete pini;
	pini = new Ini("/home/lvuser/robot.ini");
}

// SUPERESTRUCTURE ACCESSORS AND MUTATORS IN ROBOTMODEL

//returns if the intake arm is down (or up)
bool RobotModel::IsIntakeArmDown() {
	return !intakeArmSolenoidB->Get();
}

//moves intake arm up
void RobotModel::MoveIntakeArmUp() {
	intakeArmSolenoidA->Set(false);
	intakeArmSolenoidB->Set(true);
}

//moves intake arm down
void RobotModel::MoveIntakeArmDown() {
	intakeArmSolenoidA->Set(true);
	intakeArmSolenoidB->Set(false);
}

//changes the state of the intake arm (i.e. if up, move down)
void RobotModel::ChangeIntakeArmState() {
	if (IsIntakeArmDown()) {
		MoveIntakeArmUp();
	} else {
		MoveIntakeArmDown();
	}
}

//returns the speed of the intake motor
double RobotModel::GetIntakeMotorSpeed() {
	return intakeMotor->Get();
}

//sets the speed of the intake motor
void RobotModel::SetIntakeMotorSpeed(double speed) {
	DO_PERIODIC(20, printf("Set intake speed to %f\n", speed));
	intakeMotor->Set(speed);
}

//returns if the intake switch is up (or down)
bool RobotModel::GetIntakeSwitchState() {
	return intakeSwitch->Get();
}

//returns if the defense manipulator is down (or up)
bool RobotModel::IsDefenseManipDown() {
	return !defenseManipSolenoidB->Get();
}

// moves the defense manipulator up
void RobotModel::MoveDefenseManipUp() {
	defenseManipSolenoidA->Set(false);
	defenseManipSolenoidB->Set(true);
}

// moves the defense manipulator down
void RobotModel::MoveDefenseManipDown() {
	DO_PERIODIC(1, printf("Move defense arm down\n"));
	defenseManipSolenoidA->Set(true);
	defenseManipSolenoidB->Set(false);
}

//changes the state of the defense manipulator (i.e. if up, move down)
void RobotModel::ChangeDefenseManipState() {
	if (IsDefenseManipDown()) {
		MoveDefenseManipUp();
	} else {
		MoveDefenseManipDown();
	}
}

//returns the speed of the outake motor
double RobotModel::GetOuttakeMotorSpeed() {
	return outtakeMotorA->Get();
}

//sets the speed of the outake motor
void RobotModel::SetOuttakeMotorSpeed(double speed) {
	outtakeMotorA->Set(-speed);
	outtakeMotorB->Set(speed);
}

//returns the distance of the outake motor encoder
double RobotModel::GetOuttakeEncoderVal() {
	return outtakeEncoder1->Get() - outtakeEncoder2->Get() /2;
}

//resets the outake motor encoder
void RobotModel::ResetOuttakeEncoders() {
	outtakeEncoder1->Reset();
	outtakeEncoder2->Reset();
}

//stops the compressor
void RobotModel::SetCompressorStop() {
	compressor->Stop();
}

//returns if the brake is on or off
bool RobotModel::GetBrake() {
	return brakeSolenoidA->Get();
}

//puts the break on
void RobotModel::SetBrakeOn() {
	brakeSolenoidA->Set(true);
	brakeSolenoidB->Set(false);
}

//takes the break off
void RobotModel::SetBrakeOff() {
	brakeSolenoidA->Set(false);
	brakeSolenoidB->Set(true);
}

// return the camera image
Image* RobotModel::GetCameraImage() {
// if the camera is on the robot, return the camera frame
// else if the USB camera is on, return the USB Camera frame
#if USE_CAMERA
//	camera->GetImage(frame);
//	return frame;
	return NULL;
#elif USE_USB_CAMERA
	usbCamera->GetImage(usbFrame);
	return usbFrame;
#else
	return NULL;
#endif

}
