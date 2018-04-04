#include "Handler.h"
// --
#include <GUILib\InteractiveWidget.h>
#include <MathLib\Ray.h>

// TODO: NewHandler which passes P3D's to the handlers.

// NOTE: offset is local2world

Handler::Handler() {
}
 
P3D Handler::get_xy0(double xPos, double yPos) {
	Ray ray = InteractiveWidget::getRayFromScreenCoords(xPos, yPos);
	P3D o = ray.origin;
	V3D d = ray.direction;
	// (o + d*f).z = 0
	double f = -o[2]/d[2];
	return o + d*f; 
}

P3D Handler::get_xyz(double xPos, double yPos, GLTrackingCamera *camera, Plane *plane) {
	Ray ray = InteractiveWidget::getRayFromScreenCoords(xPos, yPos);
	// --
	Quaternion Q = camera->getCameraRotation();
	// TODO: camera->getCameraPosition()
	P3D p2 = (P3D) Q.rotate(plane->p);
	V3D n2 = (P3D) Q.rotate(plane->n);
	Plane *plane2 = new Plane(p2, n2);
	// --
	P3D xyz; ray.getDistanceToPlane(*plane2, &xyz); // (*)
	return xyz;
}
