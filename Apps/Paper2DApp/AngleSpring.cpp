#include "AngleSpring.h"
#include <GUILib/GLUtils.h>
#include <FEMSimLib/SimulationMesh.h>

AngleSpring::AngleSpring(SimulationMesh* simMesh, Node* n1, Node* n2, Node* n3) : SimMeshElement(simMesh) {
	this->n[0] = n1;
	this->n[1] = n2;
	this->n[2] = n3;

	setRestShapeFromCurrentConfiguration();
}

AngleSpring::~AngleSpring(){}

void AngleSpring::setRestShapeFromCurrentConfiguration() {
	P3D p1 = n[0]->getCoordinates(this->simMesh->X);
	P3D p2 = n[1]->getCoordinates(this->simMesh->X);
	P3D p3 = n[2]->getCoordinates(this->simMesh->X);

	restAngle = getAngle(this->simMesh->X);
}

double AngleSpring::getMass() {
	return 0;
}

double AngleSpring::getAngle(const dVector& x) {
	P3D p1 = n[0]->getCoordinates(x);
	P3D p2 = n[1]->getCoordinates(x);
	P3D p3 = n[2]->getCoordinates(x);

	double res = atan2(p3[1] - p2[1], p3[0] - p2[0]) - atan2(p2[1] - p1[1], p2[0] - p1[0]);
	return res;
}

double AngleSpring::getEnergy(const dVector& x, const dVector& X) {
	double d_angle = getAngle(x) - restAngle;
	if (d_angle > PI) d_angle -= 2 * PI;
	if (d_angle < -PI)d_angle += 2 * PI;
	return 0.5*d_angle*d_angle*k;
}

void AngleSpring::addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) {
	P3D p1 = n[0]->getCoordinates(x);
	P3D p2 = n[1]->getCoordinates(x);
	P3D p3 = n[2]->getCoordinates(x);
	double dx_a = p2[0] - p1[0];
	double dy_a = p2[1] - p1[1];
	double dx_b = p3[0] - p2[0];
	double dy_b = p3[1] - p2[1];
	double d2_a = dx_a * dx_a + dy_a * dy_a;
	double d2_b = dx_b * dx_b + dy_b * dy_b;
	double d_angle = getAngle(x)- restAngle;
	if (d_angle > PI) d_angle -= 2 * PI;
	if (d_angle < -PI)d_angle += 2 * PI;
	grad[n[0]->dataStartIndex + 0] += k * d_angle*(-dy_a / d2_a);
	grad[n[0]->dataStartIndex + 1] += k * d_angle*(dx_a / d2_a);
	grad[n[1]->dataStartIndex + 0] += k * d_angle*(dy_a / d2_a + dy_b / d2_b);
	grad[n[1]->dataStartIndex + 1] += k * d_angle*(-dx_a / d2_a - dx_b / d2_b);
	grad[n[2]->dataStartIndex + 0] += k * d_angle*(-dy_b / d2_b);
	grad[n[2]->dataStartIndex + 1] += k * d_angle*(dx_b / d2_b);
}

void AngleSpring::addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries) {
	P3D p1 = n[0]->getCoordinates(x);
	P3D p2 = n[1]->getCoordinates(x);
	P3D p3 = n[2]->getCoordinates(x);
	double dx_a = p2[0] - p1[0];
	double dy_a = p2[1] - p1[1];
	double dx_b = p3[0] - p2[0];
	double dy_b = p3[1] - p2[1];
	double d2_a = dx_a * dx_a + dy_a * dy_a;
	double d2_b = dx_b * dx_b + dy_b * dy_b;
	double d_angle = getAngle(x) - restAngle;
	if (d_angle > PI) d_angle -= 2 * PI;
	if (d_angle < -PI)d_angle += 2 * PI;

	double comp_x0 = dy_a / d2_a;
	double comp_y0 = dx_a / d2_a;
	double comp_x2 = dy_b / d2_b;
	double comp_y2 = dx_b / d2_b;
	double comp_x1 = comp_x0 + comp_x2;
	double comp_y1 = comp_y0 + comp_y2;

	double comp_AD = 2 * dx_a*dy_a / (d2_a*d2_a);
	double comp_AS = (dx_a*dx_a - dy_a * dy_a) / (d2_a*d2_a);
	double comp_BD = 2 * dx_b*dy_b / (d2_b*d2_b);
	double comp_BS = (dx_b*dx_b - dy_b * dy_b) / (d2_b*d2_b);

	int x0 = n[0]->dataStartIndex;
	int y0 = n[0]->dataStartIndex+1;
	int x1 = n[1]->dataStartIndex;
	int y1 = n[1]->dataStartIndex+1;
	int x2 = n[2]->dataStartIndex;
	int y2 = n[2]->dataStartIndex+1;

	// TODO: there has to be a better way to set these

	//x0 parts
	hesEntries.push_back(MTriplet(x0, x0, k * (comp_x0*comp_x0 - d_angle * comp_AD)));
	hesEntries.push_back(MTriplet(y0, x0, k * (-comp_x0 * comp_y0 + d_angle * comp_AS)));
	hesEntries.push_back(MTriplet(x1, x0, k * (-comp_x0 * comp_x1 + d_angle * comp_AD)));
	hesEntries.push_back(MTriplet(y1, x0, k * (comp_x0 * comp_y1 - d_angle * comp_AS)));
	hesEntries.push_back(MTriplet(x2, x0, k * (comp_x0 * comp_x2)));
	hesEntries.push_back(MTriplet(y2, x0, k * (-comp_x0 * comp_y2)));
	
	//y0 parts
	hesEntries.push_back(MTriplet(y0, y0, k * (comp_y0*comp_y0 + d_angle * comp_AD)));
	hesEntries.push_back(MTriplet(x1, y0, k * (comp_y0*comp_x1 - d_angle * comp_AS)));
	hesEntries.push_back(MTriplet(y1, y0, k * (-comp_y0 * comp_y1 - d_angle * comp_AD)));
	hesEntries.push_back(MTriplet(x2, y0, k * (-comp_y0 * comp_x2)));
	hesEntries.push_back(MTriplet(y2, y0, k * (comp_y0 * comp_y2)));

	//x1 parts
	hesEntries.push_back(MTriplet(x1, x1, k * (comp_x1*comp_x1 + d_angle * (comp_BD - comp_AD))));
	hesEntries.push_back(MTriplet(y1, x1, k * (-comp_x1 * comp_y1 + d_angle * (comp_AS - comp_BS))));
	hesEntries.push_back(MTriplet(x2, x1, k * (-comp_x1 * comp_x2 - d_angle * comp_BD)));
	hesEntries.push_back(MTriplet(y2, x1, k * (comp_x1 * comp_y2 + d_angle * comp_BS)));

	//y1 parts
	hesEntries.push_back(MTriplet(y1, y1, k * (comp_y1*comp_y1 + d_angle * (comp_AD - comp_BD))));
	hesEntries.push_back(MTriplet(x2, y1, k * (comp_y1 * comp_x2 + d_angle * comp_BS)));
	hesEntries.push_back(MTriplet(y2, y1, k * (-comp_y1 * comp_y2 + d_angle * comp_BD)));
	
	//x2 parts
	hesEntries.push_back(MTriplet(x2, x2, k * (comp_x2 * comp_x2 + d_angle * comp_BD)));
	hesEntries.push_back(MTriplet(y2, x2, k * (-comp_x2 * comp_y2 - d_angle * comp_BS)));

	//y2 parts
	hesEntries.push_back(MTriplet(y2, y2, k * (comp_y2 * comp_y2 - d_angle * comp_BD)));

	// So this hessian is triangular?
}

void AngleSpring::draw(const dVector& x) {
	P3D p1 = n[0]->getCoordinates(x);
	P3D p2 = n[1]->getCoordinates(x);
	P3D p3 = n[2]->getCoordinates(x);

	P3D pa = 0.5*(p1 + p2);
	P3D pb = 0.5*(p2 + p3);

	// draw a line between the midpoints of the edges n[0]-n[1] and n[1]-n[2]
	glBegin(GL_LINES);
		glVertex3d(pa[0], pa[1], pa[2]);
		glVertex3d(pb[0], pb[1], pb[2]);
	glEnd();
}

void AngleSpring::drawRestConfiguration(const dVector& X) {

}

