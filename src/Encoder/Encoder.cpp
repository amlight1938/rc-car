#include "Encoder/Encoder.h"

Encoder::Encoder(uint8_t phaseAPin, uint8_t phaseBPin)
{
	// starting the count for each motor encoder
	encoder.attachFullQuad(phaseAPin, phaseBPin); // 4x decoding
	encoder.setCount(0);

	prevMotorCount = 0;
	lastMeasurementTime = millis();
}

double Encoder::getVelocity()
{
	unsigned long currentTime = millis();
	// reading out the encoder values
	int64_t motorCount = encoder.getCount();

	// calculating number of triggers/millisecond then scaling to triggers/second
	triggerRate = 1000.0 * (double)(motorCount - prevMotorCount) / (currentTime - lastMeasurementTime);

	prevMotorCount = motorCount;
	lastMeasurementTime = currentTime;

	// (m/s) = (trig/s) / (trig/m)
	return triggerRate / TRIGGERS_PER_METER;
}