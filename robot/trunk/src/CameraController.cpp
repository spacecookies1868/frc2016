#include "CameraController.h"
#include <math.h>
#include "Logger.h"

CameraController::CameraController(RobotModel* robot) {
	this->robot = robot;
	table = robot->gripLines;
	topFromGround = 95.0; //inches
	bottomFromGround = 83.0; //inches
	//distortion coefficients!
	/*
	 * [-0.5430259802451465; 0.270945658136815; 0.006719749057770554; 0.009482155791447877; 0.1002601217038307
	 *
	 */
	focalX = 813.4497;
	focalY = 814.8689;
	cameraCenterX = 323;
	cameraCenterY = 233.527; //DO NOT CONFUSE WITH OTHER CENTERX AND CENTERY!!!

	targetHeight = 12; //inches
	leftDisFromCenter = 10; //inches, fake goal
	rightDisFromCenter = 10; //inches, fake goal
	x = 0;
	y = 0;
	leftR = 0.0;
	rightR = 0.0;

	cameraAngle = 3.1415926535897932 * 18 / 180; //subject to change * 5/36
	heightOfCamera = 14.5; //inches subject to change
	targetLength2 = 20; //inches

}

void CameraController::Reset() {
	table->Reset();
	x = 0;
	y = 0;
}

void CameraController::CalculateDistanceWithAngles(bool leftTargetDesired) {
	table->ReadValues(leftTargetDesired);
	printf("Read Table Values \n");
	printf("Top Left (%f, %f) \n", table->GetTopLeftX(), table->GetTopLeftY());
	printf("Top Right (%f, %f)\n", table->GetTopRightX(), table->GetTopRightY());
	printf("Bottom Left (%f, %f)\n", table->GetBottomLeftX(), table->GetBottomLeftY());
	printf("Bottom Right (%f, %f)\n", table->GetBottomRightX(), table->GetBottomRightY());
	leftR = CalculateRadiusWithTriangles(true);
	rightR = CalculateRadiusWithTriangles(false);
	double leftx = (table->GetTopLeftX() + table->GetBottomLeftX()) / 2;
	double rightx = ((table->GetTopRightX() + table->GetBottomRightX())) / 2;
	printf("image average x (left, right) : (%f,%f)\n", leftx, rightx);
	leftx = leftx - cameraCenterX;
	rightx = rightx - cameraCenterX;
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

	x = (rightXOffset + leftXOffset)/2.0; //used to be targetCenter
	y = (leftYdis + rightYdis)/2.0;

	printf("(x,y): (%f,%f) \n", x, y);
	DUMP("Left Radius: ", leftR);
	DUMP("Right Radius: ", rightR);
	DUMP("Left Angle: ", leftAngle);
	DUMP("Right Angle: ", rightAngle);
	DUMP("Seen Target Length: ", targetLength);
	DUMP("X: ",x);
	DUMP("Y: ",y);

}

void CameraController::CalculateDistance(bool leftTargetDesired) {
	table->ReadValues(leftTargetDesired);
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
	double realX = (imageX - cameraCenterX) / focalX;
	double realY = (imageY - cameraCenterY) / focalY;
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

double CameraController::CalculateRadiusWithTriangles(bool leftRadiusDesired) {
	if(leftRadiusDesired) {
		printf("95 - height of camera %f \n", topFromGround - heightOfCamera);
		printf("atan stuff %f \n", atan((cameraCenterY - table->GetTopLeftY())/focalY));
		printf("tan stuff %f \n", tan(atan((cameraCenterY - table->GetTopLeftY())/focalY)+cameraAngle));
		return (topFromGround-heightOfCamera)/tan(atan((cameraCenterY - table->GetTopLeftY())/focalY)+cameraAngle);
	}
	else {
		printf("95 - height of camera %f \n", topFromGround - heightOfCamera);
		printf("atan stuff %f \n", atan((cameraCenterY - table->GetTopRightY())/focalY));
		printf("tan stuff %f \n", tan(atan((cameraCenterY- table->GetTopRightY())/focalY)+cameraAngle));
		return (topFromGround-heightOfCamera)/tan(atan((cameraCenterY - table->GetTopRightY())/focalY)+cameraAngle);
	}
}

