#include "LowerSimplex.h"
#include "CSTElement3D.h"
#include "SimulationMesh.h"
#include <PlushHelpers\tetgen.h>
#include <PlushHelpers\error.h>

class SimulationMesh;
 
LowerSimplex::LowerSimplex(vector<Node *> nodes, SimMeshElement* higher_simplex) {
	this->nodes = nodes;
	this->higher_simplex = higher_simplex;
}
 
LowerSimplex::~LowerSimplex() { }

int LowerSimplex::D() {
	return nodes.size() - 1;
}

void LowerSimplex::draw(const dVector &y) {
	glPushAttrib(GL_ALL_ATTRIB_BITS); {
		glLineWidth(6.);
		glPointSize(6.); 
		// --
		if (D() == 1) {
			glDisable(GL_CULL_FACE);
			set_color(HENN1NK);
			glBegin(GL_LINE_STRIP); {
				for (auto &node : nodes) {
					glP3D(node->getCoordinates(y));
				} 
			} glEnd();
		// --
		} else if (D() == 2) {
			// (2) Front faces	
			glEnable(GL_CULL_FACE);
			glCullFace(GL_BACK);
			glBegin(GL_TRIANGLES); {
				// Triangle orientation using shewchuck predicates on parent tet
				vector<P3D> tri = {nodes[0]->getCurrentPosition(),
				                   nodes[1]->getCurrentPosition(),
				                   nodes[2]->getCurrentPosition() };
 
				P3D test;
				for (auto &node : dynamic_cast<CSTElement3D *>(higher_simplex)->n) { 
					if (node->nodeIndex != nodes[0]->nodeIndex &&
						node->nodeIndex != nodes[1]->nodeIndex &&
						node->nodeIndex != nodes[2]->nodeIndex) {
						// --
						test = node->getCurrentPosition();
					} 
				}
				double orientation_ = orient3d(tri[0].data(), tri[1].data(), tri[2].data(), test.data());
				bool oriented = (orientation_ > 0.);
				// --
				V3D n;
				// V3D X01 = V3D(nodes[0]->getUndeformedPosition(), nodes[1]->getUndeformedPosition());
				// V3D X02 = V3D(nodes[0]->getUndeformedPosition(), nodes[2]->getUndeformedPosition());
				V3D X01 = V3D(nodes[0]->getCurrentPosition(), nodes[1]->getCurrentPosition());
				V3D X02 = V3D(nodes[0]->getCurrentPosition(), nodes[2]->getCurrentPosition());
				if (oriented) {
					n = X01.cross(X02);
				} else {
					n = X02.cross(X01);
				}
				n.normalize();
				// // --
				P3D COL_N = (P3D) (n + V3D(1, 1, 1)).normalized(); // https://en.wikipedia.org/wiki/Normal_mapping
				// V3D rgb = color_swirl(.2, WHITE, COL_N);
				V3D rgb = COL_N;
				double a = 1.;
				if (this->higher_simplex->simMesh->DRAW_BOUNDARY_TRANSPARENT) {
					a = .3;
				}
				glColor4f((GLfloat) rgb[0], (GLfloat) rgb[1], (GLfloat) rgb[2], (GLfloat) a);
				// --
				if (oriented) {
					glP3D(tri[0]);
					glP3D(tri[1]);
					glP3D(tri[2]);
				} else {
					glP3D(tri[1]);
					glP3D(tri[0]);
					glP3D(tri[2]); 
				}
			} glEnd(); 
		// --
		} else {
			error("That's not a lower simplex.");
		}
	}

	glPopAttrib();
}
/*
	glPushAttrib(GL_ALL_ATTRIB_BITS); {
		glLineWidth(5.);
		glPointSize(5.);

		glBegin(GL_LINES); {
			for (int i = 0; i < N; ++i) {
				auto &edge = boundary_simplices[i];
				double f0 = double(i) / double(N);
				double f1 = double(i + 1) / double(N);
				set_color_swirl(f0, RED, GREEN);
				// set_color(BLACK);
				glP3Dz(edge->nodes[0]->getCurrentPosition(), 5);
				set_color_swirl(f1, RED, GREEN);
				// set_color(WHITE);
				glP3Dz(edge->nodes[1]->getCurrentPosition(), 5);
			}
		} glEnd();

		glBegin(GL_POINTS); {
			for (int i = 0; i < N; ++i) {
				auto &edge = boundary_simplices[i];
				set_color(CYAN);
				glP3Dz(edge->nodes[0]->getCurrentPosition(), 5);
			}
		} glEnd();
	} glPopAttrib();
*/