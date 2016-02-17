#include "CameraController.h"
#include <math.h>
#include "Logger.h"

CameraController::CameraController(RobotModel* robot) {
	this->robot = robot;
	table = robot->gripLines;
//	focalX = 404.5;
//	focalY = 403.37;
//	centerX = 155.32;
//	centerY = 126.32;
	focalX = 809;
	focalY = 806.74;
	centerX = 310.64;
	centerY = 252.64;
	targetHeight = 17.75; //inches, fake goal
	leftDisFromCenter = 12; //inches, fake goal
	rightDisFromCenter = 12; //inches, fake goal
	x = 0;
	y = 0;
	PI = 3.14159265358979;
}

void CameraController::Reset() {
	table->Reset();
	x = 0;
	y = 0;
}

void CameraController::SetX(double newX) {
	x = newX;
}

void CameraController::SetY(double newY) {
	y = newY;
}

void CameraController::CalculateDistanceWithAngles() {
	table->ReadValues();
	printf("Read Table Values \n");

	leftR = CalculateRadius(table->GetTopLeftX(), table->GetTopLeftY(),
			table->GetBottomLeftX(), table->GetBottomLeftY());
	rightR = CalculateRadius(table->GetTopRightX(), table->GetTopRightY(),
			table->GetBottomRightX(), table->GetBottomRightY());
	double leftx = (table->GetTopLeftX() + table->GetBottomLeftX()) / 2;
	double rightx = ((table->GetTopRightX() + table->GetBottomRightX())) / 2;
	printf("image average x (left, right) : (%f,%f)\n", leftx, rightx);
	leftx = leftx - centerX;
	rightx = rightx - centerX;
	printf("normalled x (left, right) : (%f, %f)\n", leftx, rightx);
	// RADIANS BTW
	double leftAngle = atan(leftx / focalX);
	double rightAngle = atan(rightx / focalX);

	printf("angles (radians) (left, right): (%f, %f)\n", leftAngle, rightAngle);

	double leftXOffset = leftR * sin(leftAngle);
	double rightXOffset = rightR * sin(rightAngle);
	printf("X offset (left, right): (%f, %f)\n", leftXOffset, rightXOffset);
	double leftYdis = leftR * cos(leftAngle);
	double rightYdis = rightR * cos(rightAngle);
	printf("Y dis (left, right): (%f, %f) \n", leftYdis, rightYdis);
	double targetLength = sqrt(pow((rightXOffset - leftXOffset),2) + pow((leftYdis - rightYdis), 2));
	double targetCenter = targetLength / 2;
	printf("Target length %f\n", targetLength);
	printf("Target center %f\n", targetCenter);

	x = rightXOffset - targetCenter;
	y = (leftYdis + rightYdis)/2;

	printf("(x,y): (%f,%f) \n", x, y);
	DUMP("Left Radius: ", leftR);
	DUMP("Right Radius: ", rightR);
	DUMP("Left Angle: ", leftAngle);
	DUMP("Right Angle: ", rightAngle);
	DUMP("Seen Target Length: ", targetLength);
	DUMP("X: ",x);
	DUMP("Y: ",y);

}

void CameraController::CalculateDistance() {
	table->ReadValues();
	printf("Read Table Values \n");

	leftR = CalculateRadius(table->GetTopLeftX(), table->GetTopLeftY(),
									table->GetBottomLeftX(), table->GetBottomLeftY());
	rightR = CalculateRadius(table->GetTopRightX(), table->GetTopRightY(),
									table->GetBottomRightX(), table->GetBottomRightY());
//	double leftR = CalculateRadius(table->GetTopLeftX(), table->GetTopLeftY(),
//			table->GetTopLeftX(), table->GetBottomLeftY());
//	double rightR = CalculateRadius(table->GetTopRightX(),
//			table->GetTopRightY(), table->GetTopRightX(),
//			table->GetBottomRightY());
	printf("LeftR %f\n", leftR);
	printf("RightR %f\n", rightR);

	x = ((rightR*rightR) - (leftR*leftR) - (leftDisFromCenter*leftDisFromCenter - rightDisFromCenter*rightDisFromCenter)) / (2*(leftDisFromCenter + rightDisFromCenter));
	y = sqrt(rightR*rightR - (x + rightDisFromCenter)*(x + rightDisFromCenter));
	printf("x %f y %f \n", x, y);
}

double CameraController::GetX() {
	return x;
}

double CameraController::GetY() {
	return y;
}

std::vector<double> CameraController::CalculateRealCoords(double imageX, double imageY) {
	/*
	 * Very basic, not dealing with radial distortion
	 */
	std::vector<double> realCoords = std::vector<double>();
	//double realX = (imageX - centerX) / focalX;
	//double realY = (imageY - centerY) / focalY;
	double realX = (imageX - centerX) / focalX;
	double realY = (imageY - centerY) / focalY;
//	printf("Calculating COORDINATES \n");
//	printf("imageX %f, imageY %f \n", imageX, imageY);
//	printf("realX %f, realY %f \n", realX, realY);
//	printf("------------------------------------\n");
	realCoords.push_back(realX);
	realCoords.push_back(realY);
	return realCoords;
}

double CameraController::CalculateSlope(double imageX, double imageY) {
	std::vector<double> realCoords = CalculateRealCoords(imageX, imageY);
	double realX = realCoords[0];
	double realY = realCoords[1];

	double XZdis = sqrt((realX*realX) + 1);
	//double XZdis = 1.045;
	double slope = realY / XZdis;
//	printf("Calculating SLOPE \n");
//	printf("realX %f, realY %f \n", realX, realY);
//	printf("XZ distance %f\n", XZdis);
//	printf("slope %f\n", slope);
//	printf("---------------------------------------\n");
	return slope;
}

double CameraController::CalculateRadius(double imageX1, double imageY1, double imageX2, double imageY2) {
	double slope1 = CalculateSlope(imageX1, imageY1);
	double slope2 = CalculateSlope(imageX2, imageY2);
	double radius  = fabs(targetHeight / (slope1 - slope2));
//	printf("Calculating RADIUS \n");
//	printf("Slope1 %f, Slope2 %f\n", slope1, slope2);
//	printf("targetHeight %f, slopeDiff %f\n", targetHeight, (slope1-slope2));
//	printf("Radius %f\n", radius);
//	printf("-----------------------------------------\n");
	return radius;
}
