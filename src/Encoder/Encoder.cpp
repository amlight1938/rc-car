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
	double newVelocity = triggerRate / TRIGGERS_PER_METER;

	// using the deque to calculate a rolling average over the last DEQUE_MAX_SIZE steps to reduce noise
	if (rollingAverageDeque.size() >= DEQUE_MAX_SIZE)
	{
		// remove the front and subtract it from the rolling sum then add the new velocity
		const auto &oldVelocity = rollingAverageDeque.front();
		currentDequeSum -= oldVelocity;
		rollingAverageDeque.pop_front();
		rollingAverageDeque.push_back(newVelocity);
		currentDequeSum += newVelocity;
	}
	else
	{
		// not at capacity yet so just add the new velocity
		rollingAverageDeque.push_back(newVelocity);
		currentDequeSum += newVelocity;
	}

	// returning the average velocity of the previous DEQUE_MAX_SIZE measurements
	return currentDequeSum / rollingAverageDeque.size(); // m/s
}