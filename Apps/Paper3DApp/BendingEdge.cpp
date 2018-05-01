#include "BendingEdge.h"
#include <GUILib/GLUtils.h>
#include <FEMSimLib/SimulationMesh.h>

BendingEdge::BendingEdge(SimulationMesh* simMesh, Node* n1, Node* n2, Node* n3, Node* n4) : SimMeshElement(simMesh) {
	this->n[0] = n1;
	this->n[1] = n2;
	this->n[2] = n3;
	this->n[3] = n4;

	setRestShapeFromCurrentConfiguration();
}

BendingEdge::~BendingEdge(){}

void BendingEdge::setRestShapeFromCurrentConfiguration() {
	P3D x0 = n[0]->getWorldPosition();
	P3D x1 = n[1]->getWorldPosition();
	P3D x2 = n[2]->getWorldPosition();
	P3D x3 = n[3]->getWorldPosition();

	restAngle = 0;//TODO: maybe somebody wants something besides 0
	restEdgeLength = (x1 - x0).norm();
	restArea = 0.5 * ((x0 - x2).cross(x1 - x2).norm() + (x1 - x3).cross(x0 - x3).norm());
}

double BendingEdge::getMass() {
	return 0;
}

double BendingEdge::getAngle(const dVector& x) {
	P3D x0 = n[0]->getCoordinates(x);
	P3D x1 = n[1]->getCoordinates(x);
	P3D x2 = n[2]->getCoordinates(x);
	P3D x3 = n[3]->getCoordinates(x);

	P3D n1 = (x0 - x2).cross(x1 - x2).normalized();
	P3D n2 = (x1 - x3).cross(x0 - x3).normalized();

	return acos(n1.dot(n2));
}

double BendingEdge::getEnergy(const dVector& x, const dVector& X) {
	double d_angle = getAngle(x) - restAngle;
	return k * 3 * restEdgeLength*restEdgeLength * d_angle*d_angle / restArea;
}

void BendingEdge::addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) {
	//define various values needed
	P3D x0 = n[0]->getCoordinates(x);
	P3D x1 = n[1]->getCoordinates(x);
	P3D x2 = n[2]->getCoordinates(x);
	P3D x3 = n[3]->getCoordinates(x);
	double d_angle = getAngle(x) - restAngle;
	P3D e0 = x1 - x0;
	P3D e1 = x2 - x0;
	P3D e2 = x3 - x0;
	P3D e3 = x2 - x1;
	P3D e4 = x3 - x1;
	P3D n1 = e1.cross(e3);
	P3D n2 = e4.cross(e2);
	double l_e0 = e0.norm(); e0 /= l_e0;
	double l_e1 = e1.norm(); e1 /= l_e1; 
	double l_e2 = e2.norm(); e2 /= l_e2;
	double l_e3 = e3.norm(); e3 /= l_e3;
	double l_e4 = e4.norm(); e4 /= l_e4;
	double l_n1 = n1.norm(); n1 /= l_n1;
	double l_n2 = n2.norm(); n2 /= l_n2;
	double angle_1 = acos(e0.dot(e1));
	double angle_2 = acos(e2.dot(e0));
	double angle_3 = acos(e3.dot(-e0));
	double angle_4 = acos((-e0).dot(e4));
	double h_1 = l_n1 / l_e1;
	double h_2 = l_n2 / l_e2;
	double h_3 = l_n1 / l_e3;
	double h_4 = l_n2 / l_e4;
	double h_01 = l_n1 / l_e0;
	double h_02 = l_n2 / l_e0;

	// constant factor depending only on rest shape
	double K = k * 6 * restEdgeLength*restEdgeLength / restArea;
	// combine with factor depending on current angle
	double ccc = K * d_angle;//d_angle will be replaced when using phi, psi

	P3D component;
	// dE/dx0
	component = (n1*cos(angle_3) / h_3 + n2 * cos(angle_4) / h_4)*ccc;
	for (int i = 0;i < 3;++i)grad[n[0]->dataStartIndex + i] += component[i];
	// dE/dx1
	component = (n1*cos(angle_1) / h_1 + n2 * cos(angle_2) / h_2)*ccc;
	for (int i = 0;i < 3;++i)grad[n[1]->dataStartIndex + i] += component[i];
	// dE/dx2
	component = n1 * (-ccc) / h_01;
	for (int i = 0;i < 3;++i)grad[n[2]->dataStartIndex + i] += component[i];
	// dE/dx3
	component = n2 * (-ccc) / h_02;
	for (int i = 0;i < 3;++i)grad[n[3]->dataStartIndex + i] += component[i];
}

void BendingEdge::addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries) {
	//TODO
}

void BendingEdge::draw(const dVector& x) {
}

void BendingEdge::drawRestConfiguration(const dVector& X) {
}

