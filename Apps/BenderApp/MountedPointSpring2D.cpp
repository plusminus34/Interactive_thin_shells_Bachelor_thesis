#include <GUILib/GLUtils.h>

#include "MountedPointSpring2D.h"


MountedPointSpring2D::MountedPointSpring2D(SimulationMesh * simMesh, Node * node, P3D referencePosition, Mount * mount)
	: FixedPointSpring2D(simMesh, node, referencePosition), mount(mount)
{}


MountedPointSpring2D::~MountedPointSpring2D()
{}


double MountedPointSpring2D::getEnergy(const dVector & x, const dVector & X){
	if(!mount->active) {return(0.0);};
	//E = 0.5K xTx
	P3D mountedTargetPosition = mount->getTransformedX(targetPosition);
	return 0.5 * K * (node->getCoordinates(x)-mountedTargetPosition).dot(node->getCoordinates(x)-mountedTargetPosition);
}

void MountedPointSpring2D::addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) {
	if(!mount->active) {return;}
	//compute the gradient, and write it out
	//dEdx = Kx
	P3D mountedTargetPosition = mount->getTransformedX(targetPosition);
	for (int j = 0; j < 2; j ++) {
		grad[node->dataStartIndex + j] += K * (node->getCoordinates(x)[j] - mountedTargetPosition[j]);
	}
}

void MountedPointSpring2D::addEnergyHessianTo(const dVector & x, const dVector & X, std::vector<MTriplet>& hesEntries){
	if(!mount->active) {return;}
	//ddEdxdx = I;
	Matrix2x2 ddEdxdx;
	ddEdxdx << 1, 0, 0, 1;
	addSparseMatrixDenseBlockToTriplet(hesEntries, node->dataStartIndex, node->dataStartIndex, K * ddEdxdx, true);
}


void MountedPointSpring2D::addDeltaFDeltaXi(std::vector<dVector> & dfdxi)
{
	if(!mount->active) {return;}

	double K = this->K;

	std::vector<V3D> dfdxi_temp;

	mount->getDxDpar(mount->getTransformedX(targetPosition), dfdxi_temp);

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



void MountedPointSpring2D::draw(const dVector& x) {
	if(mount->active) {
		// draw line to current position
		glColor3d(1, 0, 0);
		P3D pi =(node->getCoordinates(x));
		P3D pj = mount->getTransformedX(targetPosition);
		glBegin(GL_LINES);
		glVertex3d(pi[0], pi[1], 0);
		glVertex3d(pj[0], pj[1], 0);
		glEnd();
		// draw node
		//glColor3d(0.6, 0, 1);
		//drawSphere(node->getWorldPosition(), 0.008);
	}
}

void MountedPointSpring2D::draw(const dVector& x, double size, double r, double g, double b) {
	/*
	if(mount->active) {
		// draw line to current position
		glColor3d(1, 0.5, 0.5);
		P3D pi =(node->getCoordinates(x));
		P3D pj = mount->getTransformedX(targetPosition);
		glBegin(GL_LINES);
		glVertex3d(pi[0], pi[1], 0);
		glVertex3d(pj[0], pj[1], 0);
		glEnd();
	}
	*/
		// draw node
		glColor3d(r, g, b);
		drawSphere(node->getWorldPosition(), size);
}

