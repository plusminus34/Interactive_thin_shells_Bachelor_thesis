#include "MountedPointSpring2D.h"


MountedPointSpring2D::MountedPointSpring2D(SimulationMesh * simMesh, Node * node, P3D referencePosition, Mount * mount)
	: FixedPointSpring2D(simMesh, node, referencePosition), mount(mount)
{}


MountedPointSpring2D::~MountedPointSpring2D()
{}


double MountedPointSpring2D::getEnergy(const dVector & x, const dVector & X){
	//E = 0.5K xTx
	P3D mountedTargetPosition = mount->getTransformedX(targetPosition);
	return 0.5 * K * (node->getCoordinates(x)-mountedTargetPosition).dot(node->getCoordinates(x)-mountedTargetPosition);
}

void MountedPointSpring2D::addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) {
	//compute the gradient, and write it out
	//dEdx = Kx
	P3D mountedTargetPosition = mount->getTransformedX(targetPosition);
	for (int j = 0; j < 2; j ++) {
		grad[node->dataStartIndex + j] += K * (node->getCoordinates(x)[j] - mountedTargetPosition[j]);
	}
}

void MountedPointSpring2D::addEnergyHessianTo(const dVector & x, const dVector & X, std::vector<MTriplet>& hesEntries){
	//ddEdxdx = I;
	Matrix2x2 ddEdxdx;
	ddEdxdx << 1, 0, 0, 1;
	addSparseMatrixDenseBlockToTriplet(hesEntries, node->dataStartIndex, node->dataStartIndex, K * ddEdxdx, true);
}