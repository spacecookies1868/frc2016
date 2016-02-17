#include "TableReader.h"
#include <math.h>
#include "Logger.h"

TableReader::TableReader(llvm::StringRef tableName) {
	table = NetworkTable::GetTable(tableName);
	sumLeftX1 = 0;
	sumLeftY1 = 0;
	sumLeftX2 = 0;
	sumLeftY2 = 0;
	sumRightX1 = 0;
	sumRightY1 = 0;
	sumRightX2 = 0;
	sumRightY2 = 0;
	numIterations = 20;
	leftLineIndex = 0;
	rightLineIndex = 0;

//values assuming top right corner is 0,0
//	topLeftX = 0;
//	topLeftY = 240;
//	topRightX = 320;
//	topRightY = 240;
//	bottomLeftX = 0;
//	bottomLeftY = 0;
//	bottomRightX = 320;
//	bottomRightY = 0;

//values top left corner is 0,0
//	topRightX = 0;
//	topRightY = 240;
//	topLeftX = 320;
//	topLeftY = 240;
//	bottomRightX = 0;
//	bottomRightY = 0;
//	bottomLeftX = 320;
//	bottomLeftY = 0;
//
//	topRightX = 0;
//	topRightY = 360;
//	topLeftX = 480;
//	topLeftY = 360;
//	bottomRightX = 0;
//	bottomRightY = 0;
//	bottomLeftX = 480;
//	bottomLeftY = 0;

	topRightX = 0;
	topRightY = 480;
	topLeftX = 640;
	topLeftY = 480;
	bottomRightX = 0;
	bottomRightY = 0;
	bottomLeftX = 640;
	bottomLeftY = 0;

}

void TableReader::ReadValues() {
	lengths = table->GetNumberArray("length", llvm::ArrayRef<double>());
	x1 = table->GetNumberArray("x1", llvm::ArrayRef<double>());
	x2 = table->GetNumberArray("x2", llvm::ArrayRef<double>());
	y1 = table->GetNumberArray("y1", llvm::ArrayRef<double>());
	y2 = table->GetNumberArray("y2", llvm::ArrayRef<double>());
	angles = table->GetNumberArray("angle", llvm::ArrayRef<double>());

//	for (unsigned int i = 0; i < lengths.size(); i++) {
//		printf("line %i: \n", i + 1);
//		printf("length :%f \n", lengths[i]);
//		printf("x1 :%f y1: %f x2: %f y2: %f\n", x1[i], y1[i], x2[i], y2[i]);
//		printf("angle %f \n", angles[i]);
//	}

	//IterativeFilterLines();
	FilterLines();
}

void TableReader::Reset() {
	printf("Resetting \n");
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

int TableReader::GetLeftLineIndex() {
	return leftLineIndex;
}

int TableReader::GetRightLineIndex() {
	return rightLineIndex;
}

double TableReader::GetLeftLineX1() {
	return sumLeftX1 / numIterations;
	//return x1[leftLineIndex];
}

double TableReader::GetLeftLineX2() {
	return sumLeftX2 / numIterations;
	//return x2[leftLineIndex];
}

double TableReader::GetLeftLineY1() {
	return sumLeftY1 / numIterations;
	//return y1[leftLineIndex];
}

double TableReader::GetLeftLineY2() {
	return sumLeftY2 / numIterations;
	//return y2[leftLineIndex];
}

double TableReader::GetLeftLineLength() {
	return lengths[leftLineIndex];
}

double TableReader::GetRightLineX1() {
	return sumRightX1 / numIterations;
	//return x1[rightLineIndex];
}

double TableReader::GetRightLineX2() {
	return sumRightX2 / numIterations;
	//return x2[rightLineIndex];
}

double TableReader::GetRightLineY1() {
	return sumRightY1 / numIterations;
	//return y1[rightLineIndex];
}

double TableReader::GetRightLineY2() {
	return sumRightY2 / numIterations;
	//return y2[rightLineIndex];
}

double TableReader::GetRightLineLength() {
	return lengths[rightLineIndex];
}
void TableReader::FilterLines() {
	for (unsigned int i = 0; i < lengths.size(); i++) {
//		printf("beginning iteration %i\n", i);
//		printf("My Current values: \n");
//		printf("Top Left (x,y): (%f,%f)\n", topLeftX, topLeftY);
//		printf("Bottom Left (x,y): (%f,%f)\n", bottomLeftX, bottomLeftY);
//		printf("Top Right (x,y): (%f,%f)\n", topRightX, topRightY);
//		printf("Bottom Right (x,y): (%f,%f)\n", bottomRightX, bottomRightY);
		if (fabs((double) angles[i]) < 45.0
				|| fabs((double) angles[i]) > 135.0) {
			//printf("Horizontal Line %i\n", i);
		} else {
			//printf("Vertical Line %i\n", i);
			//ALL CALCS BASED ON TOP LEFT IS (0,0)
			double currx = x1[i];
			double curry = y1[i];
			if ((currx < topLeftX && curry < topLeftY)
					|| (fabs(currx - topLeftX) < 5 && curry < topLeftY)
					|| ((topLeftX - currx) > 9 && fabs(curry - topLeftY) < 10)) {
				printf("line %i, (x1, y1) replacing top Left: (%f, %f)\n", i,
						currx, curry);
				topLeftX = currx;
				topLeftY = curry;
			}
			if (((currx < bottomLeftX && curry > bottomLeftY)
					|| (fabs(currx - bottomLeftX) < 5 && curry > bottomLeftY)
					|| ((bottomLeftX - currx) > 9
							&& fabs(curry - bottomLeftY) < 10)) && fabs(topLeftX - currx) < 10) {
				printf("line %i, (x1, y1) replacing bottom Left: (%f, %f)\n", i,
						currx, curry);
				bottomLeftX = currx;
				bottomLeftY = curry;
			}
			if ((currx > topRightX && curry < topRightY)
					|| (fabs(currx - topRightX) < 5 && curry < topRightY)
					|| ((currx - topRightX) > 9 && fabs(curry - topRightY) < 10)) {
				printf("line %i, (x1, y1) replacing top Right: (%f, %f)\n", i,
						currx, curry);
				topRightX = currx;
				topRightY = curry;
			}
			if (((currx > bottomRightX && curry > bottomRightY)
					|| (fabs(currx - bottomRightX) < 5 && curry > bottomRightY)
					|| ((currx - bottomRightX) > 9
							&& fabs(curry - bottomRightY) < 10)) && fabs(topRightX - currx) < 10) {
				printf("line %i, (x1, y1) replacing bottom Right: (%f, %f)\n",
						i, currx, curry);
				bottomRightX = currx;
				bottomRightY = curry;
			}
			/* testing x2, y2 */
			currx = x2[i];
			curry = y2[i];
			//using threshold of 5 pixels to be actually same edge
			if ((currx < topLeftX && curry < topLeftY)
					|| (fabs(currx - topLeftX) < 5 && curry < topLeftY)
					|| ((topLeftX - currx) > 9 && fabs(curry - topLeftY) < 10)) {
				printf("line %i, (x1, y1) replacing top Left: (%f, %f)\n", i,
						currx, curry);
				topLeftX = currx;
				topLeftY = curry;
			}
			if (((currx < bottomLeftX && curry > bottomLeftY)
					|| (fabs(currx - bottomLeftX) < 5 && curry > bottomLeftY)
					|| ((bottomLeftX - currx) > 9
							&& fabs(curry - bottomLeftY) < 10))
					&& fabs(topLeftX - currx) < 10) {
				printf("line %i, (x1, y1) replacing bottom Left: (%f, %f)\n", i,
						currx, curry);
				bottomLeftX = currx;
				bottomLeftY = curry;
			}
			if ((currx > topRightX && curry < topRightY)
					|| (fabs(currx - topRightX) < 5 && curry < topRightY)
					|| ((currx - topRightX) > 9 && fabs(curry - topRightY) < 10)) {
				printf("line %i, (x1, y1) replacing top Right: (%f, %f)\n", i,
						currx, curry);
				topRightX = currx;
				topRightY = curry;
			}
			if (((currx > bottomRightX && curry > bottomRightY)
					|| (fabs(currx - bottomRightX) < 5 && curry > bottomRightY)
					|| ((currx - bottomRightX) > 9
							&& fabs(curry - bottomRightY) < 10))
					&& fabs(topRightX - currx) < 10) {
				printf("line %i, (x1, y1) replacing bottom Right: (%f, %f)\n",
						i, currx, curry);
				bottomRightX = currx;
				bottomRightY = curry;
			}

		}
	}
	printf("Ending filter lines \n");
	printf("My values: \n");
	printf("Top Left (x,y): (%f,%f)\n", topLeftX, topLeftY);
	printf("Bottom Left (x,y): (%f,%f)\n", bottomLeftX, bottomLeftY);
	printf("Top Right (x,y): (%f,%f)\n", topRightX, topRightY);
	printf("Bottom Right (x,y): (%f,%f)\n", bottomRightX, bottomRightY);
	DUMP("Top Left X ", topLeftX);
	DUMP("Top Left Y", topLeftY);
	DUMP("Bottom Left X ", bottomLeftX);
	DUMP("Bottom Left Y ", bottomLeftY);
	DUMP("Top Right X ", topRightX);
	DUMP("Top Right Y ", topRightY);
	DUMP("Bottom Right X ", bottomRightX);
	DUMP("Bottom Right Y ", bottomRightY);
}


/*
 * Old Filter Lines Method
void TableReader::FilterLines() {
	printf("Num of Lines %i\n", angles.size());
	int leftIndex = 0;
	int rightIndex = 0;
	for (unsigned int i = 0; i < angles.size(); i++) {
		printf("%i\n",i);
		if (fabs((double)angles[i]) < 45.0 || fabs((double)angles[i]) > 135.0) {
			printf("Horizontal Line %i\n", i);
		} else {
			printf("Vertical Line %i\n", i);
			if (x1[i] > x1[rightIndex]) {
				rightIndex = i;
			}
			if (x1[i] < x1[leftIndex]) {
				leftIndex = i;
			}
		}
	}
	printf("Left %i Right %i\n", leftIndex, rightIndex);
	leftLineIndex = leftIndex;
	rightLineIndex = rightIndex;
}
*/

void TableReader::IterativeFilterLines() {
	printf("Num of Lines %i\n", angles.size());
	int leftIndex = 0;
	int rightIndex = 0;
	for (unsigned int i = 0; i < angles.size(); i++) {
		printf("%i\n", i);
		if (fabs((double) angles[i]) < 45.0
				|| fabs((double) angles[i]) > 135.0) {
			printf("Horizontal Line %i\n", i);
		} else {
			printf("Vertical Line %i\n", i);
			if (x1[i] > x1[rightIndex]) {
				rightIndex = i;
			}
			if (x1[i] < x1[leftIndex]) {
				leftIndex = i;
			}
		}
	}
	printf("Left %i Right %i\n", leftIndex, rightIndex);
	sumLeftX1 += x1[leftIndex];
	sumLeftY1 += y1[leftIndex];
	sumLeftX2 += x2[leftIndex];
	sumLeftY2 += y2[leftIndex];
	sumRightX1 += x1[rightIndex];
	sumRightY1 += y1[rightIndex];
	sumRightX2 += x2[rightIndex];
	sumRightY2 += y2[rightIndex];
}
