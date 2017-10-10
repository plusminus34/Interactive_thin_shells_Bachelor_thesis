#pragma once

#include <GUILib/GLApplication.h>
#include <string>
#include <map>



class Particle {
public:
	double mass = 1.0;
	//in pixels...
	double size = 7;

	P3D pos;
	V3D vel;

	bool isSeedParticle = false;

	double dragCoefficient = 0.01;

	double intensity = 1.0;
	double coolingRate = 0.5;

	void advanceInTime(double dt) {
		//F = mg -d*v
		V3D g(0, -9.8, 0);
		V3D force = (g * mass - vel * dragCoefficient);

		//symplectic Euler
		vel += force / mass * dt;
		pos += vel * dt;

		//BE for the intensity....
		intensity = intensity / (1 - dt * -coolingRate);
	}

	void draw() {
		glPointSize((GLfloat)size);
		glColor4d(0xFE / 255.0, 0x8A / 255.0, 0x1D / 255.0, intensity);
		glBegin(GL_POINTS);
			glVertex3d(pos[0], pos[1], pos[2]);
		glEnd();
		glPointSize(1);
	}
};


/**
 * Test App Fireworks
 */
class FireworksApp : public GLApplication {
private:
	DynamicArray<Particle> particles;

public:
	// constructor
	FireworksApp();
	// destructor
	virtual ~FireworksApp(void);
	// Run the App tasks
	virtual void process();
	// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
	virtual void drawScene();
	// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
	virtual void drawAuxiliarySceneInfo();
	// Restart the application.
	virtual void restart();


	//input callbacks...

	//all these methods should returns true if the event is processed, false otherwise...
	//any time a physical key is pressed, this event will trigger. Useful for reading off special keys...
	virtual bool onKeyEvent(int key, int action, int mods);
	//this one gets triggered on UNICODE characters only...
	virtual bool onCharacterPressedEvent(int key, int mods);
	//triggered when mouse buttons are pressed
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
	//triggered when mouse moves
	virtual bool onMouseMoveEvent(double xPos, double yPos);
	//triggered when using the mouse wheel
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);

	virtual bool processCommandLine(const std::string& cmdLine);
	
	virtual void saveFile(const char* fName);
	virtual void loadFile(const char* fName);

};



