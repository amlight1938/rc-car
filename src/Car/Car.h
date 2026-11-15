#pragma once

#include "Arduino.h"
#include "Pins.h"
#include "Motor/Motor.h"
#include "Encoder/Encoder.h"
#include "CarMotionState.h"

// a singleton class pattern - we want there to only ever be one instance of this class for the entirety of the car's lifecycle
class Car
{
public:
	// singleton class pattern requires we delete the copy, move, and assignment ctors so there's no backdoor to create a second instance of Car
	Car(const Car &) = delete;
	Car &operator=(const Car &) = delete;
	Car(Car &&) = delete;
	Car &operator=(Car &&) = delete;

	// called by other classes to get the Car. Returns a reference to the Car instead of a new Car object. Static so it can be called without a Car object present
	static Car &getInstance();

	CarMotionState getCarMotionState();

	void control(double leftStickY, double rightStickX, bool ignoreDeadzone = false);

	void stop();

	void throttleToggle();
	void throttleOff();
	void throttleOn();

private:
	Car();

	// declaring the destructor to be private so it cannot be called externally
	// however, we want the destructor to still behave in the default manner
	~Car() = default;

	static constexpr double TRACK_WIDTH = 0.145; // meters

	Motor leftMotor;
	Motor rightMotor;
	Encoder leftEncoder;
	Encoder rightEncoder;
};