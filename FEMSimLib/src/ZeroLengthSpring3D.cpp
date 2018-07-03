#include <FEMSimLib\ZeroLengthSpring3D.h>

ZeroLengthSpring3D::ZeroLengthSpring3D(SimulationMesh* simMesh, Node* n1, Node* n2) : SimMeshElement(simMesh) {

    this->n[0] = n1;
    this->n[1] = n2;
}

ZeroLengthSpring3D::~ZeroLengthSpring3D() {
}

double ZeroLengthSpring3D::getMass() {
	return 0;
}

V3D ZeroLengthSpring3D::getCurrentEdgeVector(const dVector& x, const dVector& X) {
	return V3D(n[1]->getCoordinates(x), n[0]->getCoordinates(x));
}

double ZeroLengthSpring3D::getCurrentEdgeLength(const dVector& x, const dVector& X) {
	return getCurrentEdgeVector(x, X).length();
}

double ZeroLengthSpring3D::getEnergy(const dVector& x, const dVector& X) {
	double l = getCurrentEdgeLength(x, X);
	return 0.5 * k * l * l;
}

void ZeroLengthSpring3D::addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) {
	//compute the gradient, and write it out
	computeGradientComponents(x, X);
	for (int i = 0; i<2; i++)
		for (int j = 0; j<3; j++)
			grad[n[i]->dataStartIndex + j] += dEdx[i][j];
}

void ZeroLengthSpring3D::addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries) {
	computeHessianComponents();
	for (int i = 0; i<2; i++)
		for (int j = 0; j < 2; j++)
			addSparseMatrixDenseBlockToTriplet(hesEntries, n[i]->dataStartIndex, n[j]->dataStartIndex, ddEdxdx[i][j], true);
}

void ZeroLengthSpring3D::draw(const dVector& x) {
}

void ZeroLengthSpring3D::drawRestConfiguration(const dVector& X) {
}

void ZeroLengthSpring3D::computeGradientComponents(const dVector& x, const dVector& X) {
	V3D v = getCurrentEdgeVector(x, X);
	dEdx[0] = v * k;
	dEdx[1] = - v * k;
}

void ZeroLengthSpring3D::computeHessianComponents() {
	for(int i=0;i<2;++i)
		for (int j = 0; j < 2; ++j) {
			ddEdxdx[i][j]= k*Matrix3x3::Identity();
			if (i != j) { ddEdxdx[i][j] *= -1; }
		}
}