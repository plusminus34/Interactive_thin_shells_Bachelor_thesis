#include <GUILib/GLUtils.h>

#include "MountedPointSpring.h"

#include <iostream>

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
	P3D mountedTargetPosition = mount->getTransformedX(this->targetPosition);
	return 0.5 * this->K * (this->node->getCoordinates(x)-mountedTargetPosition).dot(this->node->getCoordinates(x)-mountedTargetPosition);
}

template <int NDim>
void MountedPointSpring<NDim>::addEnergyGradientTo(const dVector& x, 
												   const dVector& X, 
												   dVector& grad) 
{
	if(!mount->active) {return;}
	//compute the gradient, and write it out
	//dEdx = Kx
	P3D mountedTargetPosition = mount->getTransformedX(this->targetPosition);
	for (int j = 0; j < NDim; j ++) {
		grad[this->node->dataStartIndex + j] += this->K * (this->node->getCoordinates(x)[j] - mountedTargetPosition[j]);
	}
}

template <int NDim>
void MountedPointSpring<NDim>::addEnergyHessianTo(const dVector & x, 
												  const dVector & X, 
												  std::vector<MTriplet>& hesEntries)
{
	if(!mount->active) {return;}
	//ddEdxdx = I;
	using TMat = typename std::conditional<NDim == 2, Matrix2x2, Matrix3x3>::type;
	TMat ddEdxdx;
	ddEdxdx.setIdentity();
	addSparseMatrixDenseBlockToTriplet(hesEntries, this->node->dataStartIndex, this->node->dataStartIndex, this->K * ddEdxdx, true);
}

template <int NDim>
void MountedPointSpring<NDim>::addDeltaFDeltaXi(MatrixNxM &dfdxi)
{
	if(!mount->active) {return;}


	double K = this->K;

	std::vector<V3D> dfdmountpar_temp;

	mount->getDxDpar(this->targetPosition, dfdmountpar_temp);


	for(V3D & gradi : dfdmountpar_temp) {
		gradi *= K;
	}

	int xi_idx_start = mount->parameters->parametersStartIndex;
	int data_idx_start = this->node->dataStartIndex;

	for(int i = 0; i < dfdmountpar_temp.size(); ++i) {
		for(int j = 0; j < NDim; ++j) {
			dfdxi(data_idx_start + j, xi_idx_start + i) += dfdmountpar_temp[i][j];
		}
	}

}


template <int NDim>
void MountedPointSpring<NDim>::draw(const dVector& x) {
	if(mount->active) {
		// draw line to current position
		glColor3d(1, 0, 0);
		P3D pi =(this->node->getCoordinates(x));
		P3D pj = mount->getTransformedX(this->targetPosition);
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
		drawSphere(this->node->getWorldPosition(), size);
}



// instantiation of 2D & 3D
// ATTENTION: other values for NDim won't work

template class MountedPointSpring<2>;
template class MountedPointSpring<3>;
