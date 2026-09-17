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
	bool dpadDown;
	bool buttonA;
	bool buttonY;
};