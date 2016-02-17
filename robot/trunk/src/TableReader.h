#ifndef TABLEREADER_H
#define TABLEREADER_H

#include "WPILib.h"


class TableReader {
public:
	TableReader(llvm::StringRef tableName);
	void ReadValues();
	void Reset();
	double GetTopLeftX();
	double GetTopLeftY();
	double GetTopRightX();
	double GetTopRightY();
	double GetBottomLeftX();
	double GetBottomLeftY();
	double GetBottomRightX();
	double GetBottomRightY();

	//old methods from iterative read
	int GetRightLineIndex();
	int GetLeftLineIndex();
	double GetLeftLineX1();
	double GetLeftLineX2();
	double GetLeftLineY1();
	double GetLeftLineY2();
	double GetLeftLineLength();
	double GetRightLineX1();
	double GetRightLineX2();
	double GetRightLineY1();
	double GetRightLineY2();
	double GetRightLineLength();
	void IterativeFilterLines();
	~TableReader() {}
private:
	void FilterLines();
	std::shared_ptr<NetworkTable> table;
	std::vector<double> lengths;
	std::vector<double> x1;
	std::vector<double> x2;
	std::vector<double> y1;
	std::vector<double> y2;
	std::vector<double> angles;
	int rightLineIndex;
	int leftLineIndex;

	double sumLeftX1, sumLeftY1, sumLeftX2, sumLeftY2;
	double sumRightX1, sumRightY1, sumRightX2, sumRightY2;
	int numIterations;

	double topLeftX, topLeftY, topRightX, topRightY, bottomLeftX, bottomLeftY, bottomRightX, bottomRightY;

};

#endif
