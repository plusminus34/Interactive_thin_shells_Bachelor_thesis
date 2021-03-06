#include <GUILib/GLUtils.h>
#include <FEMSimLib/CSTriangle3D.h>
#include <FEMSimLib/SimulationMesh.h>

#define SCALE_FACTOR 10000000.0

CSTriangle3D::CSTriangle3D(SimulationMesh* simMesh, Node* n1, Node* n2, Node* n3) : SimMeshElement(simMesh) {
//	shearModulus = 0.3 * 10e9 / SCALE_FACTOR;
//	bulkModulus = 1.5 * 10e9 / SCALE_FACTOR;

	this->n[0] = n1;
	this->n[1] = n2;
	this->n[2] = n3;

	setRestShapeFromCurrentConfiguration();

	shearModulus = 0.3;
	bulkModulus = 1.5;

	//distribute the mass of this element to the nodes that define it...
	for (int i = 0;i<3;i++)
		n[i]->addMassContribution(getMass() / 3.0);

	matModel = MM_STVK;

}

CSTriangle3D::~CSTriangle3D(){
}

void CSTriangle3D::setRestShapeFromCurrentConfiguration(){
	//edge vectors
	V3D V1(n[0]->getWorldPosition(), n[1]->getWorldPosition()), V2(n[0]->getWorldPosition(), n[2]->getWorldPosition());
	//matrix that holds three edge vectors
	Matrix2x2 dX;
	dX << V1[0], V2[0],
		V1[1], V2[1];// Warning: Rest shape is assumed to be in the plane

	dXInv = dX.inverse();//TODO .inverse() is baaad

	//compute the area of the element...
	restShapeArea = computeRestShapeArea(this->simMesh->X);
}


double CSTriangle3D::getMass() {
	return restShapeArea * massDensity;
}

double CSTriangle3D::computeRestShapeArea(const dVector& X) {
	P3D p1 = n[0]->getCoordinates(X);
	P3D p2 = n[1]->getCoordinates(X);
	P3D p3 = n[2]->getCoordinates(X);
	V3D V1(p1, p2), V2(p1, p3);
	//now compute the area of the element...
	return 1 / 2.0 * fabs(V1.cross(V2).length());
}

void CSTriangle3D::addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) {
	//compute the gradient, and write it out
	computeGradientComponents(x, X);
	for (int i = 0;i<3;i++)
		for (int j = 0;j<3;j++)
		grad[n[i]->dataStartIndex + j] += dEdx[i][j];
}

void CSTriangle3D::addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries) {
    //compute the hessian blocks and 
    computeHessianComponents(x, X);
    for (int i = 0;i<3;i++)
        for (int j = 0;j < 3;j++)
            addSparseMatrixDenseBlockToTriplet(hesEntries, n[i]->dataStartIndex, n[j]->dataStartIndex, ddEdxdx[i][j], true);
}

void CSTriangle3D::draw(const dVector& x) {

	double dn = 0.00005;
	P3D p[3];
	for (int idx = 0; idx < 3; idx++) {
		p[idx] = n[idx]->getCoordinates(x);
	}
	V3D normal = V3D(p[1], p[0]).cross(V3D(p[2], p[0])).normalized();

	glBegin(GL_TRIANGLES);
	glColor3d(1.0, 0.75, 0.75);//light red front side
		glNormal3d(normal[0], normal[1], normal[2]);
		for (int idx = 0; idx < 3; idx++) {
			glVertex3d(p[idx][0] + dn * normal[0], p[idx][1] + dn * normal[1], p[idx][2] + dn * normal[2]);
		}
	glColor3d(0.75, 0.75, 1.0);//light blue back side
		glNormal3d(-normal[0], -normal[1], -normal[2]);
		for (int idx = 0; idx < 3; idx++) {
			glVertex3d(p[idx][0] - dn * normal[0], p[idx][1] - dn * normal[1], p[idx][2] - dn * normal[2]);
		}
	glEnd();

	glColor3d(0.5, 0.5, 0.5);
	glBegin(GL_LINES);
	for (int i = 0; i < 2; i++)
		for (int j = i + 1; j < 3; j++) {
			P3D pi = p[i];
			P3D pj = p[j];
			glVertex3d(pi[0], pi[1], pi[2]);
			glVertex3d(pj[0], pj[1], pj[2]);
		}
	glEnd();
}

void CSTriangle3D::drawRestConfiguration(const dVector& X) {
	glColor3d(0, 0, 0);
	glBegin(GL_LINES);
	for (int i = 0; i<2;i++)
		for (int j = i + 1;j<3;j++) {
			P3D pi = n[i]->getCoordinates(X);
			P3D pj = n[j]->getCoordinates(X);
			glVertex3d(pi[0], pi[1], pi[2]);
			glVertex3d(pj[0], pj[1], pj[2]);
		}
	glEnd();
}

/**
	F maps deformed vectors dx to undeformed coords dX: dx = F*dX (or, in general F = dx/dX. By writing x as a weighted combination of 
	node displacements, where the weights are computed using basis/shape functions, F can be computed anywhere inside the element).
	For linear basis functions, F is constant throughout the element so an easy way to compute it is by looking at the matrix  that 
	maps deformed triangle edges to their underformed counterparts: v = FV, where v and V are matrices containing edge vectors 
	in deformed and undeformed configurations
*/
void CSTriangle3D::computeDeformationGradient(const dVector& x, const dVector& X, Matrix3x2& dxdX){
	//edge vectors
	V3D v1(n[0]->getCoordinates(x), n[1]->getCoordinates(x)), v2(n[0]->getCoordinates(x), n[2]->getCoordinates(x));
	dx << v1[0], v2[0],
		  v1[1], v2[1],
		  v1[2], v2[2];
	dxdX = dx * dXInv;

}

//implements the StVK material model
double CSTriangle3D::getEnergy(const dVector& x, const dVector& X){
	//compute the deformation gradient
	computeDeformationGradient(x, X, F);
	double energyDensity = 0;

	if (matModel == MM_STVK){
		//compute the Green Strain = 1/2 * (F'F-I)
		strain = F.transpose() * F; strain(0, 0) -= 1; strain(1, 1) -= 1; strain *= 0.5;

		//add the deviatoric part of the energy, which penalizes the change in the shape of the element - the frobenius norm of E [tr (E'E)] measures just that
		energyDensity += shearModulus * strain.squaredNorm();

		//and the volumetric/hydrostatic part, which is approximated as the trace of E and aims to maintain a constant volume
		energyDensity += bulkModulus/2 * (strain(0,0) + strain(1,1)) * (strain(0,0) + strain(1,1));
	}else if (matModel == MM_LINEAR_ISOTROPIC){
		// not implemented
	}else if (matModel == MM_NEO_HOOKEAN){
		// not implemented
	}

	return energyDensity * restShapeArea;

}

void CSTriangle3D::computeGradientComponents(const dVector& x, const dVector& X){
	//compute the gradient of the energy using the chain rule: dE/dx = dE/dF * dF/dx. dE/dF is the first Piola-Kirchoff stress sensor, for which nice expressions exist.
	//compute the deformation gradient
	computeDeformationGradient(x, X, F);
	dEdF.setZero();
	if (matModel == MM_STVK){
        strain = F.transpose() * F; strain(0, 0) -= 1; strain(1, 1) -= 1;
        strain *= 0.5;
		Matrix2x2 stress = 2 * shearModulus * strain;
		stress(0, 0) += bulkModulus * (strain(0, 0) + strain(1, 1));
		stress(1, 1) += bulkModulus * (strain(0, 0) + strain(1, 1));
        dEdF = F * stress;
	}else if (matModel == MM_LINEAR_ISOTROPIC){
		// not implemented
	}else if (matModel == MM_NEO_HOOKEAN){
		// not implemented
	}

	//dF/dx is going to be some +/- Xinv terms. The forces on nodes 1,2 can be writen as: dE/dF * XInv', while the force on node 0 is -f1-f2;
	dEdx[1] = V3D(dEdF(0, 0) * dXInv(0, 0) + dEdF(0, 1) * dXInv(0, 1), dEdF(1, 0) * dXInv(0, 0) + dEdF(1, 1) * dXInv(0, 1), dEdF(2, 0)*dXInv(0, 0) + dEdF(2, 1)*dXInv(0, 1)) * restShapeArea;
	dEdx[2] = V3D(dEdF(0, 0) * dXInv(1, 0) + dEdF(0, 1) * dXInv(1, 1), dEdF(1, 0) * dXInv(1, 0) + dEdF(1, 1) * dXInv(1, 1), dEdF(2, 0)*dXInv(1, 0) + dEdF(2, 1)*dXInv(1, 1)) * restShapeArea;
	dEdx[0] = -dEdx[1]-dEdx[2];
}

void CSTriangle3D::computeHessianComponents(const dVector& x, const dVector& X) {
    /*
	Most of this is simply copied from CSTElement2D.
	 The only thing that's different is that some matrices are 3x2 or 3x3 as opposed to 2x2.
    H = dfdx = ddEdxdx.
    H = restShapeArea * dPdx(F; dFdx) * transpose(dXInv)
    There are different formula of dPdx in different models. See below.
    dFdx = dDsdx * dXInv
    dDs = [	dx1 - dx0, dx2 - dx0
            dy1 - dy0, dy2 - dy0
			dz1 - dz0, dz2 - dz0 ] (that is one extra row not in the 2D case)
    let dx0,dy0,dx1,dy1,dx2,dy2 be 1 respectively (while others keep 0 so that we get dDsd(xk)),
    we can calculate 6 elements of H in each turn ( {l = 0..5}ddEd(xk)d(xl) ), and finally
    fill out whole H matrix. (9 3x3 matrices as opposed to the 2x2 matrices of the 2D case)
    */
    if (matModel == MM_STVK)
    {
        /*
        dPdx(F; dFdx) = dFdx * (2 * shearModulus * E + bulkModulus * trace(E) * I) +
        F * (2 * shearModulus * dEdx + bulkModulus * trace(dEdx) * I)
        dEdx = 0.5 * (transpose(dFdx) * F + transpose(F) * dFdx)
        */

        computeDeformationGradient(x, X, F);
        Matrix2x3 FT = F.transpose();
        Matrix2x2 E = 0.5 * (FT * F);
        E(0, 0) -= 0.5; E(1, 1) -= 0.5;
        Matrix2x2 I;
        I(0, 0) = I(1, 1) = 1; I(0, 1) = I(1, 0) = 0;
        for (int i = 0;i < 3;++i)
            for (int j = 0;j < 3;++j)
            {
                Matrix3x2 dFdXij;
                dFdXij(0, 0) = dFdXij(0, 1) = dFdXij(1, 0) = dFdXij(1, 1) = dFdXij(2,0) = dFdXij(2, 1) = 0;
                if (i > 0)
                    dFdXij(j, i - 1) = 1;
                else
                    dFdXij(j, 0) = dFdXij(j, 1) = -1;
                dFdXij = dFdXij * dXInv;
                Matrix2x2 dEdXij = 0.5 * (dFdXij.transpose() * F + FT * dFdXij);
                Matrix3x2 dPdXij = dFdXij * (2.0 * shearModulus * E + bulkModulus * E.trace() * I);
                dPdXij += F * (2.0 * shearModulus * dEdXij + bulkModulus * dEdXij.trace() * I);
                Matrix3x2 dHdXij = restShapeArea * dPdXij * dXInv.transpose();
                for (int ii = 0;ii < 2;++ii)
                    for (int jj = 0;jj < 3;++jj)
                    ddEdxdx[ii + 1][i](jj, j) = dHdXij(jj, ii);
                ddEdxdx[0][i](0, j) = -dHdXij(0, 1) - dHdXij(0, 0);
                ddEdxdx[0][i](1, j) = -dHdXij(1, 1) - dHdXij(1, 0);
				ddEdxdx[0][i](2, j) = -dHdXij(2, 1) - dHdXij(2, 0);
            }
        //Logger::consolePrint("%lf %lf\n%lf %lf\n", ddEdxdx[0][1](0, 0), ddEdxdx[0][1](0, 1), ddEdxdx[0][1](1, 0), ddEdxdx[1][1](1, 1));
    }
    else if (matModel == MM_LINEAR_ISOTROPIC)
	{
		// not implemented
    }
    else if (matModel == MM_NEO_HOOKEAN)
    {
        // not implemented
    }
}