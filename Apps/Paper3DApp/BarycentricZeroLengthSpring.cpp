#include "BarycentricZeroLengthSpring.h"
#include <GUILib/GLUtils.h>

BarycentricZeroLengthSpring::BarycentricZeroLengthSpring(SimulationMesh* simMesh, Node* n11, Node* n12, Node* n13, Node* n21, Node* n22, Node* n23) : SimMeshElement(simMesh) {

    this->n[0] = n11;
    this->n[1] = n12;
	this->n[2] = n13;
	this->n[3] = n21;
	this->n[4] = n22;
	this->n[5] = n23;

	for (int i = 0; i < 6; ++i) weight[i] = 0.333333;
}

BarycentricZeroLengthSpring::~BarycentricZeroLengthSpring() {
}

void BarycentricZeroLengthSpring::setWeights(int triangle, double w1, double w2, double w3) {
	weight[3 * triangle] = w1;
	weight[3 * triangle + 1] = w2;
	weight[3 * triangle + 2] = w3;
}

double BarycentricZeroLengthSpring::getMass() {
	return 0;
}

P3D BarycentricZeroLengthSpring::getCurrentPoint(int point, const dVector& x, const dVector& X) {
	P3D res(0, 0, 0);
	for (int i = 0; i < 3; ++i) {
		res += n[3 * point + i]->getCoordinates(x) * weight[3 * point + i];
	}
	return res;
}


V3D BarycentricZeroLengthSpring::getCurrentEdgeVector(const dVector& x, const dVector& X) {
	return V3D(getCurrentPoint(1, x, X), getCurrentPoint(0, x, X));
}

double BarycentricZeroLengthSpring::getCurrentEdgeLength(const dVector& x, const dVector& X) {
	return getCurrentEdgeVector(x, X).length();
}

double BarycentricZeroLengthSpring::getEnergy(const dVector& x, const dVector& X) {
	double l = getCurrentEdgeLength(x, X);
	return 0.5 * k * l * l;
}

void BarycentricZeroLengthSpring::addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) {
	//compute the gradient, and write it out
	computeGradientComponents(x, X);
	for (int i = 0; i<6; i++)
		for (int j = 0; j<3; j++)
			grad[n[i]->dataStartIndex + j] += dEdx[i][j];
}

void BarycentricZeroLengthSpring::addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries) {
	computeHessianComponents();
	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++)
			addSparseMatrixDenseBlockToTriplet(hesEntries, n[i]->dataStartIndex, n[j]->dataStartIndex, ddEdxdx[i][j], true);
}

void BarycentricZeroLengthSpring::draw(const dVector& x) {
	/*
	P3D pi = getCurrentPoint(0, x, x);
	P3D pj = getCurrentPoint(1, x, x);

	glBegin(GL_LINES);
		glVertex3d(pi[0], pi[1], pi[2]);
		glVertex3d(pj[0], pj[1], pj[2]);
	glEnd();
	*/
}

void BarycentricZeroLengthSpring::drawRestConfiguration(const dVector& X) {
}

void BarycentricZeroLengthSpring::computeGradientComponents(const dVector& x, const dVector& X) {
	V3D v = getCurrentEdgeVector(x, X);
	for (int i = 0; i < 3; ++i) {
		dEdx[i] = v * k * weight[i];
		dEdx[3 + i] = -v * k * weight[3 + i];
	}
}

void BarycentricZeroLengthSpring::computeHessianComponents() {
	for(int i = 0; i < 6; ++i)
		for (int j = 0; j < 6; ++j) {
			ddEdxdx[i][j] = k * weight[i] * weight[j] * Matrix3x3::Identity();
			if (i/3 != j/3) { ddEdxdx[i][j] *= -1; }
		}
}