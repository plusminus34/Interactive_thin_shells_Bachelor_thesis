#include "Pin.h"
#include <GUILib/GLUtils.h>

Pin::Pin(SimulationMesh* simMesh, ZLSpring* s1, ZLSpring* s2, ZLSpring* s3) : SimMeshElement(simMesh) {

    this->springs[0] = s1;
    this->springs[1] = s2;
	this->springs[2] = s3;

}

Pin::~Pin() {
	for (int i = 0; i < 3; ++i)
		delete springs[i];
}

void Pin::setStiffness(double k) {
	for (int i = 0; i < 3; ++i)
		springs[i]->k = k;
}

double Pin::getMass() {
	return 0;
}

double Pin::getEnergy(const dVector& x, const dVector& X) {
	double sum = 0.0;
	for (int i = 0; i < 3; ++i) sum += springs[i]->getEnergy(x, X);
	return sum;
}

void Pin::addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) {
	for (int i = 0; i < 3; ++i) springs[i]->addEnergyGradientTo(x, X, grad);
}

void Pin::addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries) {
	for (int i = 0; i < 3; ++i) springs[i]->addEnergyHessianTo(x, X, hesEntries);
}

void Pin::draw(const dVector& x) {
}

void Pin::drawRestConfiguration(const dVector& X) {
	P3D points[6];
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 2; ++j)
			points[2 * i + j] = springs[i]->getCurrentPoint(j, X, X);

	glColor3d(1, 1, 1);
	glBegin(GL_LINES);
	glVertex3d(points[0][0], points[0][1], points[0][2]);
	glVertex3d(points[1][0], points[1][1], points[1][2]);
	glEnd();
	for (int i = 0; i < 2; ++i) {
		glColor3d(i, 1, 0);
		glBegin(GL_LINE_LOOP);
		for (int j = 0; j < 3; ++j)
			glVertex3d(points[i + 2 * j][0], points[i + 2 * j][1], points[i + 2 * j][2]);
		glEnd();
	}
}
