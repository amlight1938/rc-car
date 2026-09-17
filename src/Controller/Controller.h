#pragma once

#include <BLEGamepadClient.h>
#include "ControllerState.h"

class Controller
{
public:
	Controller();

	ControllerState getControllerState();

private:
	XboxController controller;
	ControllerState lastRawControllerState;
};