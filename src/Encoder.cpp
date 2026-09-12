#include "Encoder.h"

Encoder::Encoder(uint8_t phaseApin, uint8_t phaseBpin)
{
	// starting the count for each motor encoder
	encoder.attachFullQuad(phaseApin, phaseBpin); // 4x decoding
	encoder.setCount(0);

	prevMotorCount = 0;
	lastMeasurementTime = millis();
}

double Encoder::getVelocity()
{
	unsigned long currentTime = millis();
	// reading out the encoder values 
	motorCount = encoder.getCount();

	// calculating number of triggers/millisecond then scaling to triggers/second
	triggerRate = 1000 * (motorCount - prevMotorCount) / (currentTime - lastMeasurementTime);

	prevMotorCount = motorCount;

	// (m/s) = (trig/s) / (trig/m)
	return triggerRate / TRIGGERS_PER_METER;
}