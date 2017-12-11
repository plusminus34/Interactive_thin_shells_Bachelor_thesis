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


void MountedPointSpring2D::addDeltaFDeltaXi(std::vector<dVector> & dfdxi)
{
	double K = this->K;

	std::vector<V3D> & dfdxi_temp;

	mount->getDxDpar(targetPosition, dfdxi_temp);

	for(V3D & gradi : dfdxi_temp) {
		gradi *= K;
	}

	int xi_idx_start = mount->parametersStartIndex;
	int data_idx_start = node->dataStartIndex;

	for(int i = 0; i < dfdxi_temp.size(); ++i) {
		for(int j = 0; j < 2; ++j) {
			dfdxi[xi_idx_start + i][data_idx_start + j] += dfdxi_temp[i][j];
		}
	}

}