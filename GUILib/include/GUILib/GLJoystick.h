#pragma once
#include <GUILib/GLApplication.h>


/**
* Joystick
	Struct created for connecting, getting information and abstracting it from a plugged joystick
*/

struct Joystick
{
public:
	//All the XBOX360 mapping values for both the buttons and the axes
	enum XBOX_BUTTONS
	{
		BUTTON_A = 0,
		BUTTON_B = 1,
		BUTTON_X = 2,
		BUTTON_Y = 3,
		BUTTON_LB = 4,
		BUTTON_RB = 5,
		BUTTON_BACK = 6,
		BUTTON_START = 7,
		BUTTON_AXES_LEFT = 8,
		BUTTON_AXES_RIGHT = 9,
		BUTTON_DPAD_UP = 10,
		BUTTON_DPAD_DOWN = 12,
		BUTTON_DPAD_LEFT = 13,
		BUTTON_DPAD_RIGHT = 11
	};
	enum XBOX_AXES
	{
		AXES_LEFT_LR = 0,
		AXES_LEFT_UD = 1,

		AXES_RIGHT_LR = 4,
		AXES_RIGHT_UD = 3,

		AXES_Z = 2
	};
	//Joystick attributes
	bool connected = false;
	int  joyNumOfAxis = 0;
	int  joyNumOfButtons = 0;
	int  joystickID = GLFW_JOYSTICK_1;
	std::string joystickName;
	//Buttons and Axes Values
	const float          *axes;
	const unsigned char  *buttons;
	/**
	Contructors and destructor
	*/
	Joystick() {};

	/**
	Methods
	*/

	// Function that checks if there is any Joystick connected to the computer. It gets the first one found
	// If it returns True it means the a joystick was found and it is assigned all the attributes for it
	// TODO:  allow multiple connections
	bool joystickConnection();

	// If the joystick is connected then this function updates the information of its 'buttons' states and is 'axes'
	void updateValues();

};