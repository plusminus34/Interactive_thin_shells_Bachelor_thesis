#include <LazyFEMSimLib/BilateralSpring3D.h>
#include <GUILib/GLUtils.h>
#include <LazyFEMSimLib/SimulationMesh.h>

#define SCALE_FACTOR 10000000.0

BilateralSpring3D::BilateralSpring3D(SimulationMesh* simMesh, Node* n1, Node* n2) : SimMeshElement(simMesh) {
    //	shearModulus = 0.3 * 10e9 / SCALE_FACTOR;
    //	bulkModulus = 1.5 * 10e9 / SCALE_FACTOR;

    this->n[0] = n1;
    this->n[1] = n2;

    setRestShapeFromCurrentConfiguration();

	//distribute the mass of this element to the nodes that define it...
	for (int i = 0; i<2; i++)
		n[i]->addMassContribution(getMass() / 2.0);
}

BilateralSpring3D::~BilateralSpring3D() {
}

void BilateralSpring3D::setRestShapeFromCurrentConfiguration() {
    restLength = computeRestShapeLength(this->simMesh->X);
}


double BilateralSpring3D::getMass() {
    return restLength * massDensity;
}

double BilateralSpring3D::computeRestShapeLength(const dVector& X) {
    P3D p1 = n[0]->getCoordinates(X);
    P3D p2 = n[1]->getCoordinates(X);
    V3D V1(p1, p2);
    //now compute the length of the spring...
    return V1.length();
}


V3D BilateralSpring3D::getCurrentEdgeVector(const dVector& x, const dVector& X) {
	return V3D(n[1]->getCoordinates(x), n[0]->getCoordinates(x));
}

//relative measure of deformation/stretch
double BilateralSpring3D::getEdgeStrain(const dVector& x, const dVector& X) {
	return (restLength - getCurrentEdgeLength(x, X)) / restLength;
}

double BilateralSpring3D::getCurrentEdgeLength(const dVector& x, const dVector& X) {
	return getCurrentEdgeVector(x, X).length();
}

void BilateralSpring3D::addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) {
    //compute the gradient, and write it out
    computeGradientComponents(x, X);
    for (int i = 0;i<2;i++)
        for (int j = 0;j<3;j++)
			grad[n[i]->dataStartIndex + j] += dEdx[i][j];
}

void BilateralSpring3D::addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries) {
    //compute the hessian blocks and 
    computeHessianComponents(x, X);
    for (int i = 0;i<2;i++)
        for (int j = 0;j < 2;j++)
			addSparseMatrixDenseBlockToTriplet(hesEntries, n[i]->dataStartIndex, n[j]->dataStartIndex, ddEdxdx[i][j], true);
}

void BilateralSpring3D::draw(const dVector& x) {
	P3D pi = n[0]->getCoordinates(x);
	P3D pj = n[1]->getCoordinates(x);

	glBegin(GL_LINES);
		glVertex3d(pi[0], pi[1], pi[2]);
		glVertex3d(pj[0], pj[1], pj[2]);
	glEnd();

	glPointSize(5.0);
	glBegin(GL_POINTS);
	glVertex3d(pi[0], pi[1], pi[2]);
	glVertex3d(pj[0], pj[1], pj[2]);
	glEnd();
	glPointSize(1.0);

}

void BilateralSpring3D::drawRestConfiguration(const dVector& X) {
}


double BilateralSpring3D::getEnergy(const dVector& x, const dVector& X) {
	double s = getEdgeStrain(x, X);
	//0.5 * k * s * s is the energy density. Integrate it over the whole spring to get the energy...
	return 0.5 * k * s * s * restLength;
}

void BilateralSpring3D::computeGradientComponents(const dVector& x, const dVector& X) {
	double s = getEdgeStrain(x, X);
	double l = getCurrentEdgeLength(x, X);
	V3D v = getCurrentEdgeVector(x, X);
	V3D dsdx1 = v / -l / restLength;
	V3D dsdx2 = v / l / restLength;

	dEdx[0] = dsdx1 * k * s * restLength;
	dEdx[1] = dsdx2 * k * s * restLength;
}

void BilateralSpring3D::computeHessianComponents(const dVector& x, const dVector& X) {
	double s = getEdgeStrain(x, X);
	double l = getCurrentEdgeLength(x, X);
	V3D v = getCurrentEdgeVector(x, X);
	V3D dsdx1 = v / -l / restLength;
	V3D dsdx2 = v / l / restLength;

	ddsdx1dx1 = v.outerProductWith(v); ddsdx1dx1 *= (1.0 / (l*l*l*restLength)); ddsdx1dx1 += Matrix3x3().Identity() * (-1.0 / (l*restLength)); ddsdx1dx1 *= k * s * restLength;
	ddsdx2dx2 = v.outerProductWith(v); ddsdx2dx2 *= (1.0 / (l*l*l*restLength)); ddsdx2dx2 += Matrix3x3().Identity() * (-1.0 / (l*restLength)); ddsdx2dx2 *= k * s * restLength;
	ddsdx1dx2 = v.outerProductWith(v); ddsdx1dx2 *= (-1.0 / (l*l*l*restLength)); ddsdx1dx2 += Matrix3x3().Identity() * (1.0 / (l*restLength)); ddsdx1dx2 *= k * s * restLength;

	ddEdxdx[0][0] = dsdx1.outerProductWith(dsdx1); ddEdxdx[0][0] *= k* restLength; ddEdxdx[0][0] += ddsdx1dx1;
	ddEdxdx[1][1] = dsdx2.outerProductWith(dsdx2); ddEdxdx[1][1] *= k* restLength; ddEdxdx[1][1] += ddsdx2dx2;
	ddEdxdx[0][1] = dsdx1.outerProductWith(dsdx2); ddEdxdx[0][1] *= k* restLength; ddEdxdx[0][1] += ddsdx1dx2;

	ddEdxdx[1][0] = ddEdxdx[0][1];
}

