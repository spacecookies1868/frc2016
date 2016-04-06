#ifndef TABLEREADER_H
#define TABLEREADER_H

#include "WPILib.h"

struct MyPoint {
	double x;
	double y;
};

struct MyLine {
	double slope, y_intercept;
};

class TableReader {
public:
	TableReader(llvm::StringRef lineTableName, llvm::StringRef contourTableName);
	void ReadValues(bool leftTargetDesired);
	void Reset();
	double GetTopLeftX();
	double GetTopLeftY();
	double GetTopRightX();
	double GetTopRightY();
	double GetBottomLeftX();
	double GetBottomLeftY();
	double GetBottomRightX();
	double GetBottomRightY();
	void CalculateDistance();
	void CalculateDistanceWithAngles();
	double CalculateRadiusAgainAgain(bool leftRadiusDesired);

	~TableReader() {}
private:
	double topLeftX, topLeftY, topRightX, topRightY, bottomLeftX, bottomLeftY, bottomRightX, bottomRightY;
	void FilterLines(bool leftTargetDesired);
	void FindCorners();
	void FindConvexHull();
	MyPoint findIntercept(MyLine l1, MyLine l2);
	MyLine leastSquareRegression(std::vector<MyPoint> pts);
	void splitIntoFour(std::vector<MyPoint> pts);
// filter Lines variables
	std::shared_ptr<NetworkTable> lineTable;
	std::shared_ptr<NetworkTable> contourTable;
	std::vector<MyPoint> myPoints;
	std::vector<double> lengths;
	std::vector<double> x1;
	std::vector<double> x2;
	std::vector<double> y1;
	std::vector<double> y2;
	std::vector<double> angles;
	std::vector<double> centerXs;
	std::vector<double> centerYs;
	std::vector<double> widths;
	std::vector<double> heights;
	double centerX;
	double centerY;
	double width;
	double height;
// find Corners variables
	std::vector<MyPoint> myConvexHull;
	std::vector<MyPoint> points;
	std::vector<MyPoint> topPoints;
	std::vector<MyPoint> rightPoints;
	std::vector<MyPoint> bottomPoints;
	std::vector<MyPoint> leftPoints;
	MyLine topLine;
	MyLine rightLine;
	MyLine bottomLine;
	MyLine leftLine;
	MyPoint topLeft;
	MyPoint topRight;
	MyPoint bottomLeft;
	MyPoint bottomRight;


	void printMyConvexHull();
	int findLowestY();
	static int findCounterClockwise(MyPoint firstPoint, MyPoint a, MyPoint b);
	static bool comparePoints(MyPoint a, MyPoint b);
	static double computeDistSquared(MyPoint a, MyPoint b);
//	static int inversion; //1 if y is bigger going up -1 going down
	static MyPoint firstPoint;

	bool include(std::vector<MyPoint> pts, MyPoint p);
	static void print(std::vector<MyPoint> a);
};

#endif
