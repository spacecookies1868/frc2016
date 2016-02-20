#include "WPILib.h"
#include <ControlBoard.h>

class MainProgram: public IterativeRobot
{
private:
	ControlBoard* controlBoard;

	void RobotInit()
	{
		controlBoard = new ControlBoard();
	}


	void AutonomousInit()
	{
	}

	void AutonomousPeriodic()
	{
	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		controlBoard->ReadControls();
		printf("Dial Reader: %f\n", controlBoard->GetDialPivotValue());
		SmartDashboard::PutNumber("Dial Reader", controlBoard->GetDialPivotValue());

	}

	void TestPeriodic()
	{
	}
};

START_ROBOT_CLASS(MainProgram)
