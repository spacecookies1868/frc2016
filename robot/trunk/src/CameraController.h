#ifndef CAMERACONTROLLER_H
#define CAMERACONTROLLER_H
#include "WPILib.h"
#include "TableReader.h"
#include "RobotModel.h"

class CameraController {
public:
	CameraController(RobotModel* robot);
	void CalculateDistance();
	void CalculateDistanceWithAngles();
	void Reset();
	double GetX();
	double GetY();
	void SetX(double newX);
	void SetY(double newY);
	~CameraController() {}
private:
	std::vector<double> CalculateRealCoords(double imageX, double imageY);
	double CalculateSlope(double imageX, double imageY);
	double CalculateRadius(double imageX1, double imageY1, double imageX2, double imageY2);
	TableReader* table;
	RobotModel* robot;
	//constants
	double focalX;
	double focalY;
	double centerX;
	double centerY;
	double targetHeight;
	double leftDisFromCenter;
	double rightDisFromCenter;
	double z;
	double x;
	double y;
	double leftR;
	double rightR;
	double PI;
};
#endif
