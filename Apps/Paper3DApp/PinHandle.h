#pragma once

/*
	A data structure containing all relevant information for interacting with one end of a pin
*/
struct PinHandle {
	// Id of the pin this handle belongs to
	int pin_id;
	// Which end of the pin it is (always 0 or 1)
	int index;
	// Position of the handle in 2D
	double x;
	double y;
	// Orientation
	double angle;
	bool flipped;
	// Distance between handle position and triangle corners
	double size = 0.025;

	Vector2d getPoint(int i) {
		Vector2d res;
		if (flipped) i *= -1;
		double alpha = angle + i * PI * 2.0 / 3.0;
		res << x + size * cos(alpha),
			y + size * sin(alpha);
		return res;
	}
};

