#include "MagicPoint.h"
#include "PlateMesh.h"
#include "GUILib\InteractiveWidget.h"


MagicPoint::MagicPoint(GLTrackingCamera *camera, PlateMesh *mesh, Plane *fallback_plane_) {
	this->camera = camera;
	this->mesh = mesh;
	this->fallback_plane_ = fallback_plane_; 
	// --
	SAFE_update_s(get_projection_of(s, get_fallback_plane()));
	s_prime_prev_ = s;
	// int viewport[4];
	// glGetIntegerv(GL_VIEWPORT, viewport);
	// this->curr_far_ray = getFarRayFromScreenCoords(getRandomNumberInRange(viewport[0], viewport[2]), getRandomNumberInRange(viewport[0], viewport[2]));
	// this->DROPPED = true;
}

void MagicPoint::draw() {

	if (SELECTED || DROPPED) {
	
		if (SELECTED) {
			if (!attempt_project_onto_mesh()) {
				ON_MESH = false;
				project_onto_fallback_plane();
			} else {
				ON_MESH = true;
			}
		}

		if (DROPPED) {
			DROPPED = false;
			if (attempt_project_onto_mesh()) {
				ON_MESH = true;
			} else {
				project_onto_fallback_plane();
				ON_MESH = false;
			}
		} 
	}

	if (!ON_MESH) {
		enforce_fallback();
	}

	// --

	glPushAttrib(GL_ALL_ATTRIB_BITS); {
		COLOR = (PLATE != nullptr) ? color_inverse(PLATE->get_n_color()) : WHITE;
		set_color(COLOR);
 
		glPushMatrix(); {
			shared_ptr<Plane> plane = (PLATE != nullptr) ? PLATE->get_Pi() : get_fallback_plane();
			glTranslateP3D(this->s);
			glRotateV3D2V3D(V3D(0., 0., 1.), plane->n);
			if (PLATE != nullptr) { glTranslated(0., 0., .001); }
			drawCircle(0., 0., .002); // TODO: Own version that uses glPointSize.
		} glPopMatrix();

	} glPopAttrib(); 
}

Ray MagicPoint::getFarRayFromScreenCoords(double xPos, double yPos) {
	Ray ray_ = InteractiveWidget::getRayFromScreenCoords(xPos, yPos);
	return Ray(ray_.origin - ray_.direction*100., ray_.direction);
}


P3D MagicPoint::get_intersection_of_curr_far_ray_with(shared_ptr<Plane> Pi, double &dist) {
	P3D ret; dist = curr_far_ray.getDistanceToPlane(*Pi, &ret); // (*) 
	return ret;
}

P3D MagicPoint::get_intersection_of_curr_far_ray_with(shared_ptr<Plane> Pi) {
	double _; return get_intersection_of_curr_far_ray_with(Pi, _); 
}

void MagicPoint::SAFE_update_s(const P3D &s_new) {
	this->s = s_new;
	s_prime_prev_ = camera->getCameraRotation().inverseRotate(s); 
} 

bool MagicPoint::attempt_project_onto_mesh() { 

	double min_dist = INFINITY;
	P3D min_s_Pi;
	Plate *min_PLATE = nullptr;
	bool FOUND_AT_LEAST_ONE_VALID_INTERSECTION = false;
	bool min_s_Pi_WAS_SET = false;
	for (auto &plate : mesh->plates) {

		bool FRONT_FACING = ((curr_far_ray.direction).dot(plate->get_n()) < 0.); // TODO: t-sorting
		if (!FRONT_FACING) {
			continue;
		}

		// TODO (degenerate case): bool FOUND_INTERSECTION = (!IS_ZERO(V3D(s, s_Pi).norm()));

		double dist; P3D s_Pi = get_intersection_of_curr_far_ray_with(plate->get_Pi(), dist);
 
		bool intersection_valid = plate->contains(s_Pi);
		if (intersection_valid) {
			FOUND_AT_LEAST_ONE_VALID_INTERSECTION = true;
			if (dist < min_dist) {
				min_s_Pi_WAS_SET = true;
				min_dist = dist;
				min_s_Pi = s_Pi;
				min_PLATE = plate;
			}
		}
	}

	if (FOUND_AT_LEAST_ONE_VALID_INTERSECTION) {
		if (!min_s_Pi_WAS_SET) { error("Oh no! D:"); }
		SAFE_update_s(min_s_Pi);
		PLATE = min_PLATE;
		return true;
	}
	// --
	PLATE = nullptr;
	return false;
}

bool MagicPoint::rebind() { 

	double min_dist = INFINITY;
	Plate *min_PLATE = nullptr;
	bool FOUND_AT_LEAST_ONE_VALID_INTERSECTION = false;

	for (auto &plate : mesh->plates) {

		double dist = abs(plate->get_Pi()->getSignedDistanceToPoint(s));
		P3D s_ = plate->get_Pi()->getProjectionOf(s); 
		bool intersection_valid = plate->contains(s_);

		if (intersection_valid) {
			FOUND_AT_LEAST_ONE_VALID_INTERSECTION = true;
			if (dist < min_dist) {
				min_dist = dist;
				min_PLATE = plate;
			}
		}
	}

	if (FOUND_AT_LEAST_ONE_VALID_INTERSECTION) {
		ON_MESH = true;
		PLATE = min_PLATE;
		return true;
	}
	// --
	error("rebind() is not allowed to fail.");
	ON_MESH = false;
	PLATE = nullptr;
	return false;
}

bool MagicPoint::project_onto_fallback_plane() {
	if (SELECTED) {
		P3D s_new = get_intersection_of_curr_far_ray_with(get_fallback_plane()); // (*)
		SAFE_update_s(s_new);
		return true;
	}
	return false;
}

P3D MagicPoint::get_projection_of(const P3D &s, shared_ptr<Plane> Pi) {
	return Pi->getProjectionOf(s);
}

void  MagicPoint::enforce_fallback() {
	SAFE_update_s((P3D) camera->getCameraRotation().rotate(s_prime_prev_));
}

shared_ptr<Plane> MagicPoint::get_fallback_plane() {
	P3D p_prime = (P3D) camera->getCameraRotation().rotate(fallback_plane_->p);
	V3D n_prime =       camera->getCameraRotation().rotate(fallback_plane_->n);
	return shared_ptr<Plane>(new Plane(p_prime, n_prime));
}

bool MagicPoint::intersects(FarRay ray) {
	double d = ray.getDistanceToPoint(s);
	return (d < POINT_THRESH);
}

/*
bool MagicPoint::mouse_move(double xPos, double yPos) { 
	if (SELECTED) {
		curr_far_ray = getFarRayFromScreenCoords(xPos, yPos);
		// --
		return true;
	}

	return false; 
}

bool MagicPoint::mouse_button(int button, int action, int mods, double xPos, double yPos) {
	if (action == GLFW_PRESS) {
		if (button == GLFW_MOUSE_BUTTON_LEFT) {
			curr_far_ray = getFarRayFromScreenCoords(xPos, yPos);
			// --
			LEFT_CLICKED = true;
			SELECTED = intersects(getFarRayFromScreenCoords(xPos, yPos));
		}
	} else if (action == GLFW_RELEASE) {
		LEFT_CLICKED = false;
		if (SELECTED) {
			curr_far_ray = getFarRayFromScreenCoords(xPos, yPos);
			// --
			SELECTED = false;
			DROPPED = true;
		}
	}
	return false;
} 
*/
