#include "MagicTendon.h"
#include "GUILib\InteractiveWidget.h"
#include "Frame.h"

MagicTendon::MagicTendon(GLTrackingCamera *camera, vector<MagicPoint *> points) {
	this->camera = camera;
	this->points = points;
}

void MagicTendon::draw() {
	for (auto &point : points) {
		point->draw();
	}

	for (int i = 0; i < int(points.size()) - 1; ++i) {
		int j = i + 1;
		// --
		MagicPoint *m_p = points[i];
		MagicPoint *m_q = points[j];
		// --
		P3D p = m_p->get_s();
		P3D q = m_q->get_s();
		V3D n_p = m_p->get_n();
		V3D n_q = m_q->get_n();
		V3D pq = V3D(p, q);
		double L_pq = pq.norm();
		P3D avg_pq = (p + q) * .5;
		V3D avg_n = (.5 * (n_p + n_q)).normalized();
		// --
		if (IS_ZERO(L_pq)) { continue; }
		if (!m_p->ON_MESH || !m_q->ON_MESH) {
			glMasterPush(); {
				set_color(LIGHT_CLAY);
				glLineWidth((GLfloat) 2.);
				glEnable(GL_LINE_STIPPLE);
				glLineStipple(5, 0x0101);
				glBegin(GL_LINES); {
					set_color(m_p->COLOR);
					glP3D(p);
					set_color(WHITE); // FORNOW
					glP3D(q);
				} glEnd();
			} glMasterPop();
			continue;
		}
		if (!IS_ZERO((n_p - n_q).squaredNorm())) {
			set_color(WHITE);
			glLineWidth((GLfloat) .2);
			glBalledCylinder(p, q);
			continue; 
		}
		// --
		V3D e1 = pq.normalized();
		V3D e3 = avg_n;
		P3D s = avg_pq;
		Frame object_frame = Frame(Bfrom13(e1, e3), s);
 
		glPushAttrib(GL_ALL_ATTRIB_BITS); {
			set_color(RATIONALITY);
			glLineWidth((GLfloat) .2);
	 
			glPushMatrix(); {
				object_frame.glAffineTransform();

				{
					glTranslateP3D(P3D(0., 0., .0005));
					// --
					V3D up = V3D(0., .001);
					V3D right = V3D(.001, 0.);
					V3D down = -up;
					V3D left = -right;
					// --
					P3D p_prime = P3D(-.5*L_pq, 0.);
					P3D q_prime = P3D(.5*L_pq, 0.);
					// --
					set_color(color_tint(m_p->COLOR));
					glBegin(GL_QUADS); {
						glP3D_(p_prime + up + left);
						glP3D_(p_prime + down + left);
						glP3D_(q_prime + down + right);
						glP3D_(q_prime + up + right);
					} glEnd();
				}
			} glPopMatrix();

		} glPopAttrib(); 
	}
}

bool MagicTendon::mouse_button_(int button, int action, int mods, double xPos, double yPos) {
	if (action == GLFW_PRESS) {
		if (button == GLFW_MOUSE_BUTTON_LEFT) {
			LEFT_CLICKED = true;
			// --
			Ray ray = MagicPoint::getFarRayFromScreenCoords(xPos, yPos); 
			// --
			double min_d = INFINITY;
			MagicPoint *min_point = nullptr; 
			for (auto &point : points) { 
				double d = ray.getDistanceToPoint(point->get_s());
				if (d < min_d) {
					min_d = d;
					min_point = point;
				}
			}

			if (min_d < POINT_THRESH) {
				selected_point = min_point;
				{
					selected_point->SELECTED = true;
					selected_point->curr_far_ray = MagicPoint::getFarRayFromScreenCoords(xPos, yPos);
				}
				// --
				return true;
			}

		}
	} else if (action == GLFW_RELEASE) {
		LEFT_CLICKED = false;
		if (selected_point != nullptr) {
			{
				selected_point->curr_far_ray = MagicPoint::getFarRayFromScreenCoords(xPos, yPos);
				selected_point->SELECTED = false;
				selected_point->DROPPED = true;
			}
			selected_point = nullptr;
			// --
			return true;
		}
	}
	// --
	return false;
} 

bool MagicTendon::mouse_move_(double xPos, double yPos) { 
	if (selected_point != nullptr) {
		{
			selected_point->curr_far_ray = MagicPoint::getFarRayFromScreenCoords(xPos, yPos);
		}
		return true;
	}
	return false;
}
