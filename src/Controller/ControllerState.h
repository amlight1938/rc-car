#pragma once

struct ControllerState
{
	bool connected;
	double leftStickX;
	double leftStickY;
	double rightStickX;
	double rightStickY;
	bool leftBumper;
	bool rightBumper;
	bool dpadUp;
	bool dpadDown;
	bool buttonA;
	bool buttonX;
	bool buttonY;
};