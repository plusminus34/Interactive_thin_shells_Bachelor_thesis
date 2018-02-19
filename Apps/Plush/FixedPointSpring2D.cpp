#include "FixedPointSpring2D.h"
#include <GUILib/GLUtils.h>
#include "SimulationMesh.h"

FixedPointSpring2D::FixedPointSpring2D(SimulationMesh * simMesh, Node * node, P3D targetPosition) : SimMeshElement(simMesh) {
	this->node = node;
	this->targetPosition = targetPosition;
}

FixedPointSpring2D::~FixedPointSpring2D(){
}

double FixedPointSpring2D::getEnergy(const dVector & x, const dVector & X){
	//E = 0.5K xTx
	return 0.5 * K * (node->getCoordinates(x)-targetPosition).dot(node->getCoordinates(x)-targetPosition);
}


void FixedPointSpring2D::addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) {
	//compute the gradient, and write it out
	//dEdx = Kx
	for (int j = 0; j < 2; j ++) {
		grad[node->dataStartIndex + j] += K * (node->getCoordinates(x)[j] - targetPosition[j]);
	}
}

void FixedPointSpring2D::addEnergyHessianTo(const dVector & x, const dVector & X, std::vector<MTriplet>& hesEntries){
	//ddEdxdx = I;
	Matrix2x2 ddEdxdx;
	ddEdxdx << 1, 0, 0, 1;
	addSparseMatrixDenseBlockToTriplet(hesEntries, node->dataStartIndex, node->dataStartIndex, K * ddEdxdx, true);
}

void FixedPointSpring2D::draw(const dVector & x){
	// glColor3d(1, 0, 0);
	// P3D pi = (node->getCoordinates(x));
	// P3D pj = targetPosition;
	// glBegin(GL_LINES);
	// glVertex3d(pi[0], pi[1], 0);
	// glVertex3d(pj[0], pj[1], 0);
	// glEnd();
	int Z = 0;

	glPushAttrib(GL_ALL_ATTRIB_BITS); {
		glLineWidth(4.);
		glPointSize(9.); 

		set_color(PUMPKIN);
		glBegin(GL_POINTS); {
			glP3Dz(targetPosition, Z);
		} glEnd();

		set_color(LIGHT_PUMPKIN);
		glBegin(GL_LINES); {
			glP3Dz(targetPosition, Z);
			glP3Dz(node->getCoordinates(x), Z);
		} glEnd();

	} glPopAttrib();
}
