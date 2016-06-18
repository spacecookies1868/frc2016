#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor(uint32_t myPortNumber) {
	portNumber = myPortNumber;
	ultraInput = new AnalogInput(portNumber);
}

float UltrasonicSensor::GetRangeInInches(){
	float voltage = ultraInput->GetVoltage();
	float inches = voltage/(.009766); //Number is from the sensor document
	return inches;
}

float UltrasonicSensor::GetRangeInFeet(){
	float inches = GetRangeInInches();
	return inches/12;
}

float UltrasonicSensor::GetRangeInMM(){
	float inches = GetRangeInInches();
	return inches*25.4;
}

UltrasonicSensor::~UltrasonicSensor() {
	// TODO Auto-generated destructor stub
}

