#include "Controller/Controller.h"

Controller::Controller()
{
	controller.begin();
}

ControllerState Controller::getControllerState()
{
	ControllerState returnState;		// returns calculated values for edge detection
	ControllerState rawControllerState; // stores raw controller values without calculations to be used next cycle for edge detection

	if (controller.isConnected())
	{
		XboxControlsState xboxControlsState;
		controller.read(&xboxControlsState);

		// make controller state to be returned
		returnState.connected = true;
		returnState.leftStickX = xboxControlsState.leftStickX;
		returnState.leftStickY = xboxControlsState.leftStickY;
		returnState.rightStickX = xboxControlsState.rightStickX;
		returnState.rightStickY = xboxControlsState.rightStickY;
		returnState.leftBumper = xboxControlsState.leftBumper && !lastRawControllerState.leftBumper;
		returnState.rightBumper = xboxControlsState.rightBumper && !lastRawControllerState.rightBumper;
		returnState.dpadUp = xboxControlsState.dpadUp && !lastRawControllerState.dpadUp;
		returnState.dpadDown = xboxControlsState.dpadDown && !lastRawControllerState.dpadDown;
		returnState.buttonA = xboxControlsState.buttonA && !lastRawControllerState.buttonA;
		returnState.buttonX = xboxControlsState.buttonX && !lastRawControllerState.buttonX;
		returnState.buttonY = xboxControlsState.buttonY && !lastRawControllerState.buttonY;

		// make raw controller state to be saved for next cycle edge detection
		rawControllerState.connected = true;
		rawControllerState.leftStickX = xboxControlsState.leftStickX;
		rawControllerState.leftStickY = xboxControlsState.leftStickY;
		rawControllerState.rightStickX = xboxControlsState.rightStickX;
		rawControllerState.rightStickY = xboxControlsState.rightStickY;
		rawControllerState.leftBumper = xboxControlsState.leftBumper;
		rawControllerState.rightBumper = xboxControlsState.rightBumper;
		rawControllerState.dpadUp = xboxControlsState.dpadUp;
		rawControllerState.dpadDown = xboxControlsState.dpadDown;
		rawControllerState.buttonA = xboxControlsState.buttonA;
		rawControllerState.buttonX = xboxControlsState.buttonX;
		rawControllerState.buttonY = xboxControlsState.buttonY;
	}
	else
	{
		returnState = ControllerState{}; // default/zeroed state when disconnected
		rawControllerState = ControllerState{};
	}

	lastRawControllerState = rawControllerState;

	return returnState;
}