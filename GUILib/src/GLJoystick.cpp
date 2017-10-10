#include <GUILib/GLJoystick.h>

bool Joystick::joystickConnection()
{
	bool connection = false;
	for (int ID = 0; ID < 16; ID++)
	{
		if (glfwJoystickPresent(ID) == GL_TRUE)
		{
			this->joystickID = ID;
			glfwGetJoystickAxes(ID, &this->joyNumOfAxis);
			glfwGetJoystickButtons(ID, &this->joyNumOfButtons);
			this->joystickName = glfwGetJoystickName(ID);
			connection = true;
			break;
		}
		else
		{
			this->connected = false;
			connection      = false;
		}
	}
	return connection;
}

void Joystick::updateValues()
{
	this->axes = glfwGetJoystickAxes(joystickID, &joyNumOfAxis);
	this->buttons = glfwGetJoystickButtons(joystickID, &joyNumOfButtons);

	//for (int a = 0; a < joyNumOfAxis; a++)
	//{
	//	printf("AXIS: %3d  VALUE: %4.3lf\n",a,this->axes[a]);
	//}
	//for (int b = 0; b < joyNumOfButtons; b++)
	//{
	//	printf("BUTTON: %3d  VALUE: %d\n", b, this->buttons[b]);
	//}
}
