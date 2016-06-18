#ifndef SRC_ULTRASONICSENSOR_H_
#define SRC_ULTRASONICSENSOR_H_

#include "WPILib.h"

class UltrasonicSensor {
public:
	UltrasonicSensor(uint32_t myPortNumber); //the sensor only takes up one port
	float GetRangeInInches();
	float GetRangeInFeet();
	float GetRangeInMM();
	virtual ~UltrasonicSensor();

private:
	uint32_t portNumber;
	AnalogInput* ultraInput;
};

#endif /* SRC_ULTRASONICSENSOR_H_ */
