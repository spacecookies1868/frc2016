//#ifndef SRC_DRIVESTRAIGHTCOMMAND_H_
//#define SRC_DRIVESTRAIGHTCOMMAND_H_
//
//#include <RobotModel.h>
//
//class DriveStraightCommand {
//public:
//	DriveStraightCommand(RobotModel* myRobot, double driveStraightSpeed);
//	void Init();
//	void Update(double currTimeSec, double lastTimeSec);
//	bool IsDone();
//	double GetAccumulatedYaw();
//	virtual ~DriveStraightCommand();
//private:
//	PIDConfig* CreateRPIDConfig();
//	double GetAccumulatedYaw();
//
//	RobotModel* robot;
//	PIDConfig* rPIDConfig;
//	PIDControlLoop* rPID;
//
//	double driveStraightSpeed;
//	double initialR, desiredR;
//	double currYaw, lastYaw, deltaYaw, accumulatedYaw;
//	bool isDone;
//};
//
//#endif
