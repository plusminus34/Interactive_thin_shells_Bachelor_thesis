#include <GUILib/GLUtils.h>

#include "MountedPointSpring.h"

template <int NDim>
MountedPointSpring<NDim>::MountedPointSpring(SimulationMesh * simMesh, 
											 Node * node, 
											 P3D referencePosition, 
											 Mount * mount, 
											 double K)
	: std::conditional<NDim == 2, FixedPointSpring2D, FixedPointSpring3D>::type(simMesh, node, referencePosition, K), mount(mount)
{}

template <int NDim>
MountedPointSpring<NDim>::~MountedPointSpring()
{}


template <int NDim>
double MountedPointSpring<NDim>::getEnergy(const dVector & x, 
										   const dVector & X)
{
	if(!mount->active) {return(0.0);}
	//E = 0.5K xTx
	P3D mountedTargetPosition = mount->getTransformedX(targetPosition);
	return 0.5 * K * (node->getCoordinates(x)-mountedTargetPosition).dot(node->getCoordinates(x)-mountedTargetPosition);
}

template <int NDim>
void MountedPointSpring<NDim>::addEnergyGradientTo(const dVector& x, 
												   const dVector& X, 
												   dVector& grad) 
{
	if(!mount->active) {return;}
	//compute the gradient, and write it out
	//dEdx = Kx
	P3D mountedTargetPosition = mount->getTransformedX(targetPosition);
	for (int j = 0; j < NDim; j ++) {
		grad[node->dataStartIndex + j] += K * (node->getCoordinates(x)[j] - mountedTargetPosition[j]);
	}
}

template <int NDim>
void MountedPointSpring<NDim>::addEnergyHessianTo(const dVector & x, 
												  const dVector & X, 
												  std::vector<MTriplet>& hesEntries)
{
	if(!mount->active) {return;}
	//ddEdxdx = I;
	using TMat = std::conditional<NDim == 2, Matrix2x2, Matrix3x3>::type;
	TMat ddEdxdx;
	ddEdxdx.setIdentity();
	addSparseMatrixDenseBlockToTriplet(hesEntries, node->dataStartIndex, node->dataStartIndex, K * ddEdxdx, true);
}

template <int NDim>
void MountedPointSpring<NDim>::addDeltaFDeltaXi(std::vector<dVector> & dfdxi)
{
	if(!mount->active) {return;}

	double K = this->K;

	std::vector<V3D> dfdxi_temp;

	mount->getDxDpar(mount->getTransformedX(targetPosition), dfdxi_temp);

	for(V3D & gradi : dfdxi_temp) {
		gradi *= K;
	}

	int xi_idx_start = mount->parameters->parametersStartIndex;
	int data_idx_start = node->dataStartIndex;

	for(int i = 0; i < dfdxi_temp.size(); ++i) {
		for(int j = 0; j < NDim; ++j) {
			dfdxi[xi_idx_start + i][data_idx_start + j] += dfdxi_temp[i][j];
		}
	}

}


template <int NDim>
void MountedPointSpring<NDim>::draw(const dVector& x) {
	if(mount->active) {
		// draw line to current position
		glColor3d(1, 0, 0);
		P3D pi =(node->getCoordinates(x));
		P3D pj = mount->getTransformedX(targetPosition);
		glBegin(GL_LINES);
		glVertex3d(pi[0], pi[1], pi[2]);
		glVertex3d(pj[0], pj[1], pj[2]);
		glEnd();
		// draw node
		//glColor3d(0.6, 0, 1);
		//drawSphere(node->getWorldPosition(), 0.008);
	}
}

template <int NDim>
void MountedPointSpring<NDim>::draw(const dVector& x, double size, double r, double g, double b) {
		// draw node
		glColor3d(r, g, b);
		drawSphere(node->getWorldPosition(), size);
}



// instantiation of 2D & 3D
// ATTENTION: other values for NDim won't work

template class MountedPointSpring<2>;
template class MountedPointSpring<3>;