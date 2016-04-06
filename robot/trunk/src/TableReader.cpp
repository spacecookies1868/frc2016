#include "TableReader.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include "Logger.h"

TableReader::TableReader(llvm::StringRef lineTableName, llvm::StringRef contourTableName) {
	lineTable = NetworkTable::GetTable(lineTableName);
	contourTable = NetworkTable::GetTable(contourTableName);

	topRightX = 0;
	topRightY = 480;
	topLeftX = 640;
	topLeftY = 480;
	bottomRightX = 0;
	bottomRightY = 0;
	bottomLeftX = 640;
	bottomLeftY = 0;

}

void TableReader::ReadValues(bool leftTargetDesired) {
	x1 = lineTable->GetNumberArray("x1", llvm::ArrayRef<double>());
	x2 = lineTable->GetNumberArray("x2", llvm::ArrayRef<double>());
	y1 = lineTable->GetNumberArray("y1", llvm::ArrayRef<double>());
	y2 = lineTable->GetNumberArray("y2", llvm::ArrayRef<double>());
	angles = lineTable->GetNumberArray("angle", llvm::ArrayRef<double>());
	centerXs = contourTable->GetNumberArray("centerX", llvm::ArrayRef<double>());
	centerYs = contourTable->GetNumberArray("centerY", llvm::ArrayRef<double>());
	widths = contourTable->GetNumberArray("width", llvm::ArrayRef<double>());
	heights = contourTable->GetNumberArray("height", llvm::ArrayRef<double>());

	FilterLines(leftTargetDesired);
	FindCorners();
}

void TableReader::Reset() {
	topRightX = 0;
	topRightY = 480;
	topLeftX = 640;
	topLeftY = 480;
	bottomRightX = 0;
	bottomRightY = 0;
	bottomLeftX = 640;
	bottomLeftY = 0;
}

double TableReader::GetBottomLeftX() {
	return bottomLeftX;
}
double TableReader::GetBottomLeftY() {
	return bottomLeftY;
}
double TableReader::GetBottomRightX() {
	return bottomRightX;
}
double TableReader::GetBottomRightY() {
	return bottomRightY;
}
double TableReader::GetTopLeftX() {
	return topLeftX;
}
double TableReader::GetTopLeftY() {
	return topLeftY;
}
double TableReader::GetTopRightX() {
	return topRightX;
}
double TableReader::GetTopRightY() {
	return topRightY;
}

void TableReader::FilterLines(bool leftTargetDesired) {
//post-processing stuff
//find desired contour!
/*
 * Finding the contours values of the target desired to analyze further
 */
DUMP("in filterlines", 0.0);
DUMP("size of centerXs", (double) centerXs.size());
DUMP("size of centerXs", (double) centerYs.size());
	if(centerXs.size() == 0) {
		return;
	}
	else if(centerXs.size() == 1) {
		centerX = centerXs[0];
		centerY = centerYs[0];
		width = widths[0];
		height = heights[0];
	}
	else if(centerXs[0] <= centerXs[1]) {
		if(leftTargetDesired) {
			centerX = centerXs[0];
			centerY = centerYs[0];
			width = widths[0];
			height = heights[0];
		}
		else {
			centerX = centerXs[1];
			centerY = centerYs[1];
			width = widths[1];
			height = heights[1];
		}
	}
	else {
		if (leftTargetDesired) {
			centerX = centerXs[1];
			centerY = centerYs[1];
			width = widths[1];
			height = heights[1];
		} else {
			centerX = centerXs[0];
			centerY = centerYs[0];
			width = widths[0];
			height = heights[0];
		}
	}
DUMP("in filterlines", 1.0);
/*
 * Filtering all the points found in lines report to figure out which points belong to the target desired
 */
	std::vector<int> maPoints;
	printf("width and height %f %f \n", width, height);
	printf("centerX and center Y %f %f \n", centerX, centerY);
	for (unsigned int i = 0; i < x1.size(); i++) {
//		printf("x1, y1 (%f,%f) \n", x1[i], y1[i]);
//		printf("x2, y2 (%f,%f) \n", x2[i], y2[i]);
//		printf("x1 diff, %f and width/2 + 10 %f \n", fabs(x1[i] - centerX), width/2 + 10);
//		printf("y1 diff, %f and height/2 + 10 %f \n", fabs(y1[i] - centerY), height/2 + 10);
		if (fabs(x1[i] - centerX) <= width / 2 + 10
				&& fabs(y1[i] - centerY) <= height / 2 + 10
				&& fabs(x2[i] - centerX) <= width / 2 + 10
				&& fabs(y2[i] - centerY) <= height / 2 + 10) {
			maPoints.push_back(i);
			MyPoint temp;
			temp.x = x1[i];
			temp.y = y1[i];
			myPoints.push_back(temp);
		}
	}
DUMP("in filterlines", 2.0);
}

void TableReader::FindCorners() {
	FindConvexHull();
//	printf("This is my convex hull! \n");
//	printMyConvexHull();

	splitIntoFour(myConvexHull);
	topLine = leastSquareRegression(topPoints);
	rightLine = leastSquareRegression(rightPoints);
	bottomLine = leastSquareRegression(bottomPoints);
	leftLine = leastSquareRegression(leftPoints);
	topLeft = findIntercept(topLine, leftLine);
	topRight = findIntercept(topLine, rightLine);
	bottomLeft = findIntercept(bottomLine, leftLine);
	bottomRight = findIntercept(bottomLine, rightLine);

	topLeftX = topLeft.x;
	topLeftY = topLeft.y;
	bottomLeftX = bottomLeft.x;
	bottomLeftY = bottomLeft.y;
	topRightX = topRight.x;
	topRightY = topRight.y;
	bottomRightX = bottomRight.x;
	bottomRightY = bottomRight.y;
	printf("top left (%f, %f) \n", topLeftX, topLeftY);
	printf("top right (%f, %f) \n", topRightX, topRightY);
	printf("bottom left (%f, %f) \n", bottomLeftX, bottomLeftY);
	printf("bottom right (%f, %f) \n", bottomRightX, bottomRightY);
}

void TableReader::FindConvexHull() {
	int size = myPoints.size();
	if(size < 3) {
		return;
	}
	int indexOfFirstPoint = findLowestY();
//printf("lowest index %i \n", indexOfFirstPoint);
  	std::swap(myPoints[indexOfFirstPoint], myPoints[0]);
  	firstPoint = myPoints[0];
  	std::sort(myPoints.begin() + 1, myPoints.end(), comparePoints);
//printf("sorted\n");
//print(myPoints);
  	std::vector<MyPoint> temp;
  	temp.push_back(myPoints[0]);
  	int i = 1;
  	while(i < size - 1) {
  		int orientation = findCounterClockwise(myPoints[0], myPoints[i], myPoints[i + 1]);
  		if(orientation != 0) {
  			temp.push_back(myPoints[i]);
  		}
  		i++;
  	}
  	temp.push_back(myPoints[size - 1]);
//printf("kept\n");
//print(temp);
	int size2 = temp.size();
	if(size2 < 3) {
		return;
	}

	myConvexHull.push_back(temp[0]);
	myConvexHull.push_back(temp[1]);
	myConvexHull.push_back(temp[2]);

	for(int i = 3; i < size2; i++) {
		int size3 = myConvexHull.size();
		while (-1 != findCounterClockwise(myConvexHull[size3 - 2], myConvexHull[size3 - 1], temp[i])) {
			myConvexHull.resize(size3 - 1);
			size3--;
		}
		myConvexHull.push_back(temp[i]);
	}
}


MyPoint TableReader::findIntercept(MyLine l1, MyLine l2) {
	MyPoint intercept;
	intercept.x = (l2.y_intercept - l1.y_intercept) / (l1.slope - l2.slope);
	intercept.y = l1.slope*intercept.x + l1.y_intercept;
	return intercept;
}

MyLine TableReader::leastSquareRegression(std::vector<MyPoint> pts) {
	double sumX = 0;
	double sumY = 0;
	double sumXY = 0;
	double sumXX = 0;
	double slope = 0;
	double y_intercept = 0;
	for (unsigned int i = 0; i < pts.size(); i++) {
		sumX += pts[i].x;
		sumY += pts[i].y;
		sumXY += pts[i].x * pts[i].y;
		sumXX += pts[i].x * pts[i].x;
	}
	double avgX = sumX / pts.size();
	double avgY = sumY / pts.size();

	slope = (pts.size() * sumXY - sumX * sumY) / (pts.size() * sumXX - sumX * sumX);
	y_intercept = avgY - avgX * slope;
	MyLine l;
	l.slope = slope;
	l.y_intercept = y_intercept;
//	printf("this is my line...slope: %f, y-intercept %f \n", slope, y_intercept);
	return l;
}

void TableReader::splitIntoFour(std::vector<MyPoint> pts) {
	// GOING AROUND CLOCKWISE. WILL NEED TO CHANGE IF CONVEX HULL IS COUNTERCLOCKWISE
	double PI = 3.14159265358979;
	for (unsigned int i = 0; i < pts.size(); i++) {
		MyPoint p1 = pts[i];
		MyPoint p2;
		double angle;
		double changeX, changeY;
		if(i == pts.size() - 1) {
			p2 = pts[0];
		}
		else {
			p2 = pts[i + 1];
		}
		changeX = p2.x - p1.x;
		changeY = p1.y - p2.y;
		angle = atan2(changeY, changeX);
//printf("p1 (%f, %f) p2 (%f, %f) angle %f \n", p1.x, p1.y, p2.x, p2.y, angle);
		if (angle < PI/4 && angle > -PI/4) {
			if (!include(bottomPoints, p1)) {
				bottomPoints.push_back(p1);
			}
			if (!include(bottomPoints, p2)) {
				bottomPoints.push_back(p2);
			}
		} else if (angle >= PI/4 && angle <= 3*PI/4) {
			if (!include(rightPoints, p1)) {
				rightPoints.push_back(p1);
			}
			if (!include(rightPoints, p2)) {
				rightPoints.push_back(p2);
			}
		} else if (angle >= -3*PI/4 && angle <= -PI/4) {
			if (!include(leftPoints, p1)) {
				leftPoints.push_back(p1);
			}
			if (!include(leftPoints, p2)) {
				leftPoints.push_back(p2);
			}
		} else {
			if (!include(topPoints, p1)) {
				topPoints.push_back(p1);
			}
			if (!include(topPoints, p2)) {
				topPoints.push_back(p2);
			}
		}
	}
}



/*
 * Convex Hull methods --------------------------------------------------------------------------------
 */

void TableReader::printMyConvexHull() {
	print(myConvexHull);
}

int TableReader::findLowestY() {
	int indexOfBest = 0;
	for(unsigned int i = 0; i < myPoints.size(); i++) {
		if(myPoints[i].y > myPoints[indexOfBest].y) {
			indexOfBest = i;
		}
		else if(myPoints[i].y == myPoints[indexOfBest].y && myPoints[i].x < myPoints[indexOfBest].x) {
			indexOfBest = i;
		}
	}
	return indexOfBest;
}

int TableReader::findCounterClockwise(MyPoint firstPoint, MyPoint a, MyPoint b) { //static
	double slopeThing = (a.y - firstPoint.y) * (b.x - a.x) - (a.x - firstPoint.x) * (b.y - a.y);
	slopeThing *= -1; //1 if y is bigger going up
	if(slopeThing == 0) {
		return 0;
	}
	else if(slopeThing < 0) { //counter
		return -1;
	}
	else {
		return 1;
	}
}

double TableReader::computeDistSquared(MyPoint a, MyPoint b) { //static
	return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
}

MyPoint TableReader::firstPoint;

bool TableReader::comparePoints(MyPoint a, MyPoint b) { //static
	int c = findCounterClockwise(firstPoint, a, b);
	if(c == 0) {
		return computeDistSquared(a, firstPoint) < computeDistSquared(b, firstPoint);
	}
	return c < 0;
}

bool TableReader::include(std::vector<MyPoint> pts, MyPoint p) {
	for (unsigned int i = 0; i < pts.size(); i++) {
		if (pts[i].x == p.x && pts[i].y == p.y) {
			return true;
		}
	}
	return false;
}


void TableReader::print(std::vector<MyPoint> a) { //static
	for (unsigned int i = 0; i < a.size(); i++) {
		printf("(%f, %f) \n", a[i].x, a[i].y);
	}
}
