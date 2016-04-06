#ifndef CAMERACONTROLLER_H
#define CAMERACONTROLLER_H
#include "WPILib.h"
#include "TableReader.h"
#include "RobotModel.h"

class CameraController {
public:
	CameraController(RobotModel* robot);
	void CalculateDistance(bool leftTargetDesired);
	void CalculateDistanceWithAngles(bool leftTargetDesired);
	void Reset();
	double GetX();
	double GetY();
	~CameraController() {}
private:
	std::vector<double> CalculateRealCoords(double imageX, double imageY);
	double CalculateSlope(double imageX, double imageY);
	double CalculateRadius(double imageX1, double imageY1, double imageX2, double imageY2);
	double CalculateRadiusWithTriangles(bool leftRadiusDesired);
	TableReader* table;
	RobotModel* robot;
	//constants
	double leftDisFromCenter;
	double rightDisFromCenter;
	double x;
	double y;
	double leftR;
	double rightR;
	double cameraCenterX;
	double cameraCenterY;
	double focalX;
	double focalY;
	double targetHeight;

	double topFromGround;
	double bottomFromGround;
	double cameraAngle;
	double heightOfCamera;
	double targetLength2;

};
#endif
