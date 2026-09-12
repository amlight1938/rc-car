#include "Controller/Controller.h"

Controller::Controller(){
	controller.begin();
}

ControllerState Controller::getControllerState(){
	ControllerState returnState; //returns calculated values for edge detection
	ControllerState rawControllerState; //stores raw controller values without calculations to be used next cycle for edge detection

	if (controller.isConnected()){
		XboxControlsState xBoxControlsState;
		controller.read(&xBoxControlsState);

		//make controller state to be returned 
		returnState.connected = true;
		returnState.leftStickX = xBoxControlsState.leftStickX;
		returnState.leftStickY = xBoxControlsState.leftStickY;
		returnState.rightStickX = xBoxControlsState.rightStickX;
		returnState.rightStickY = xBoxControlsState.rightStickY;
		returnState.leftBumper = xBoxControlsState.leftBumper && !lastRawControllerState.leftBumper;
		returnState.rightBumper = xBoxControlsState.rightBumper && !lastRawControllerState.rightBumper;
		returnState.dpadDown = xBoxControlsState.dpadDown && !lastRawControllerState.dpadDown;
		returnState.buttonA = xBoxControlsState.buttonA && !lastRawControllerState.buttonA;
		returnState.buttonY = xBoxControlsState.buttonY && !lastRawControllerState.buttonY;

		//make raw controller state to be saved for next cycle edge detection
		rawControllerState.connected = true;
		rawControllerState.leftStickX = xBoxControlsState.leftStickX;
		rawControllerState.leftStickY = xBoxControlsState.leftStickY;
		rawControllerState.rightStickX = xBoxControlsState.rightStickX;
		rawControllerState.rightStickY = xBoxControlsState.rightStickY;
		rawControllerState.leftBumper = xBoxControlsState.leftBumper;
		rawControllerState.rightBumper = xBoxControlsState.rightBumper;
		rawControllerState.dpadDown = xBoxControlsState.dpadDown;
		rawControllerState.buttonA = xBoxControlsState.buttonA;
		rawControllerState.buttonY = xBoxControlsState.buttonY;
	}else{
		returnState = ControllerState{};  // default/zeroed state when disconnected
		rawControllerState = ControllerState{};
	}

	lastRawControllerState = rawControllerState;

	return returnState;
}