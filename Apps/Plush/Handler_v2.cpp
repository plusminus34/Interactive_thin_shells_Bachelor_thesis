#include "Handler_v2.h"
// --
#include <GUILib\InteractiveWidget.h>
#include <MathLib\Ray.h>

Handler_v2::Handler_v2(Frame *frame) {
	// NOTE: FORNOW Assumes frame contains a 2D transformation (preserving Plane[z = 0])
	if (frame != nullptr) {
		this->frame = frame;
	}
}
 
P3D Handler_v2::get_xy0(double xPos, double yPos) {
	Ray ray = InteractiveWidget::getRayFromScreenCoords(xPos, yPos);
	P3D o = ray.origin;
	V3D d = ray.direction;
	// (o + d*f).z = 0
	double f = -o[2]/d[2];
	return frame->world2local(o + d*f);
}

// P3D Handler_v2::get_xyz(double xPos, double yPos, GLTrackingCamera *camera, Plane *plane) {
// 	Ray ray = InteractiveWidget::getRayFromScreenCoords(xPos, yPos);
// 	// --
// 	Quaternion Q = camera->getCameraRotation();
// 	// TODO: camera->getCameraPosition()
// 	P3D p2 = (P3D) Q.rotate(plane->p);
// 	V3D n2 = (P3D) Q.rotate(plane->n);
// 	Plane *plane2 = new Plane(p2, n2);
// 	// --
// 	P3D xyz; ray.getDistanceToPlane(*plane2, &xyz); // (*)
// 	return xyz;
// }
