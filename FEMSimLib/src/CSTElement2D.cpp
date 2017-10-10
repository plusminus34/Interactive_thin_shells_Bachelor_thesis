#include <GUILib/GLUtils.h>
#include <FEMSimLib/CSTElement2D.h>
#include <FEMSimLib/SimulationMesh.h>

#define SCALE_FACTOR 10000000.0

CSTElement2D::CSTElement2D(SimulationMesh* simMesh, Node* n1, Node* n2, Node* n3) : SimMeshElement(simMesh) {
//	shearModulus = 0.3 * 10e9 / SCALE_FACTOR;
//	bulkModulus = 1.5 * 10e9 / SCALE_FACTOR;

	this->n[0] = n1;
	this->n[1] = n2;
	this->n[2] = n3;

	setRestShapeFromCurrentConfiguration();

	//distribute the mass of this element to the nodes that define it...
	for (int i = 0;i<3;i++)
		n[i]->addMassContribution(getMass() / 3.0);

//	matModel = MM_LINEAR_ISOTROPIC;
//	matModel = MM_STVK;
	matModel = MM_NEO_HOOKEAN;

}

CSTElement2D::~CSTElement2D(){
}

void CSTElement2D::setRestShapeFromCurrentConfiguration(){
	//edge vectors
	V3D V1(n[0]->getWorldPosition(), n[1]->getWorldPosition()), V2(n[0]->getWorldPosition(), n[2]->getWorldPosition());
	//matrix that holds three edge vectors
	Matrix2x2 dX;
	dX << V1[0], V2[0], 
		  V1[1], V2[1];

	dXInv = dX.inverse();

	//compute the area of the element...
	restShapeArea = computeRestShapeArea(this->simMesh->X);
//	Logger::logPrint("CSTElement2D Element area: %lf\n", restShapeArea);
}


double CSTElement2D::getMass() {
	return restShapeArea * massDensity;
}

double CSTElement2D::computeRestShapeArea(const dVector& X) {
	P3D p1 = n[0]->getCoordinates(X);
	P3D p2 = n[1]->getCoordinates(X);
	P3D p3 = n[2]->getCoordinates(X);
	V3D V1(p1, p2), V2(p1, p3);
	//now compute the area of the element...
	return 1 / 2.0 * fabs(V1.cross(V2).length());
}

void CSTElement2D::addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) {
	//compute the gradient, and write it out
	computeGradientComponents(x, X);
	for (int i = 0;i<3;i++)
		for (int j = 0;j<2;j++)
		grad[n[i]->dataStartIndex + j] += dEdx[i][j];
}

void CSTElement2D::addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries) {
    //compute the hessian blocks and 
    computeHessianComponents(x, X);
    for (int i = 0;i<3;i++)
        for (int j = 0;j < 3;j++)
            addSparseMatrixDenseBlockToTriplet(hesEntries, n[i]->dataStartIndex, n[j]->dataStartIndex, ddEdxdx[i][j], true);
}

void CSTElement2D::draw(const dVector& x) {
//	glColor3d(1, 1, 1);
	glBegin(GL_LINES);
	for (int i = 0; i<2;i++)
		for (int j = i + 1;j<3;j++) {
			P3D pi = n[i]->getCoordinates(x);
			P3D pj = n[j]->getCoordinates(x);
			glVertex3d(pi[0], pi[1], pi[2]);
			glVertex3d(pj[0], pj[1], pj[2]);
		}
	glEnd();
}

void CSTElement2D::drawRestConfiguration(const dVector& X) {
	glColor3d(1, 0, 0);
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
	maps deformed traingle/tet edges to their underformed counterparts: v = FV, where v and V are matrices containing edge vectors 
	in deformed and undeformed configurations
*/
void CSTElement2D::computeDeformationGradient(const dVector& x, const dVector& X, Matrix2x2& dxdX){
	//edge vectors
	V3D v1(n[0]->getCoordinates(x), n[1]->getCoordinates(x)), v2(n[0]->getCoordinates(x), n[2]->getCoordinates(x));
	dx << v1[0], v2[0],
		  v1[1], v2[1];
	dxdX = dx * dXInv;

//	print("../out/v1.m", v1);
//	print("../out/v2.m", v2);
//	print("../out/dx.m", dx);
//	print("../out/dXInv.m", dXInv);
//	print("../out/dxdX.m", dxdX);
}

//implements the StVK material model
double CSTElement2D::getEnergy(const dVector& x, const dVector& X){
	//compute the deformation gradient
	computeDeformationGradient(x, X, F);
	double energyDensity = 0;

	if (matModel == MM_STVK){
		//compute the Green Strain = 1/2 * (F'F-I)
		strain = F * F.transpose(); strain(0,0) -= 1; strain(1,1) -= 1; strain *= 0.5;

		//add the deviatoric part of the energy, which penalizes the change in the shape of the element - the frobenius norm of E [tr (E'E)] measures just that
		energyDensity += shearModulus * (strain(0,0)*strain(0,0) + strain(0,1)*strain(0,1) + strain(1,0)*strain(1,0) + strain(1,1)*strain(1,1));

		//and the volumetric/hydrostatic part, which is approximated as the trace of E and aims to maintain a constant volume
		energyDensity += bulkModulus/2 * (strain(0,0) + strain(1,1)) * (strain(0,0) + strain(1,1));
	}else if (matModel == MM_LINEAR_ISOTROPIC){
		//compute the Cauchy strain: 1/2 (F+F') - I
		strain = (F + F.transpose()) * 0.5; strain(0, 0) -= 1; strain(1, 1) -= 1;

		//add the deviatoric part of the energy, which penalizes the change in the shape of the element - the frobenius norm of E [tr (E'E)] measures just that
		energyDensity += shearModulus * (strain(0,0)*strain(0,0) + strain(0,1)*strain(0,1) + strain(1,0)*strain(1,0) + strain(1,1)*strain(1,1));

		//and the volumetric/hydrostatic part, which is approximated as the trace of E and aims to maintain a constant volume
		energyDensity += bulkModulus/2 * (strain(0,0) + strain(1,1)) * (strain(0,0) + strain(1,1));
	}else if (matModel == MM_NEO_HOOKEAN){
		double normF2 = F(0,0)*F(0,0) + F(0,1)*F(0,1) + F(1,0)*F(1,0) + F(1,1)*F(1,1);
		double detF = F.determinant();

		energyDensity += shearModulus/2 * (normF2-2) - shearModulus * log(detF) + bulkModulus/2 * log(detF) * log(detF);
	}

	return energyDensity * restShapeArea;

}

void CSTElement2D::computeGradientComponents(const dVector& x, const dVector& X){
	//compute the gradient of the energy using the chain rule: dE/dx = dE/dF * dF/dx. dE/dF is the first Piola-Kirchoff stress sensor, for which nice expressions exist.

	//compute the deformation gradient
	computeDeformationGradient(x, X, F);
	dEdF.setZero();
	Finv = F.inverse();
	FinvT = Finv.transpose();
	if (matModel == MM_STVK){
        strain = F.transpose() * F; strain(0, 0) -= 1; strain(1, 1) -= 1;
        strain *= 0.5;
        dEdF = strain;
        dEdF *= 2.0 * shearModulus;
        dEdF(0, 0) += bulkModulus * (strain(0, 0) + strain(1, 1));
        dEdF(1, 1) += bulkModulus * (strain(0, 0) + strain(1, 1));
        dEdF = F * dEdF;
	}else if (matModel == MM_LINEAR_ISOTROPIC){
		//compute the Cauchy strain: 1/2 (F+F') - I
		strain = (F + F.transpose()) * 0.5; strain(0, 0) -= 1; strain(1, 1) -= 1;
		dEdF = strain; dEdF *= 2*shearModulus;
		dEdF(0,0) += (strain(0,0) + strain(1,1)) * bulkModulus;
		dEdF(1,1) += (strain(0,0) + strain(1,1)) * bulkModulus;
	}else if (matModel == MM_NEO_HOOKEAN){
		double normF2 = F(0,0)*F(0,0) + F(0,1)*F(0,1) + F(1,0)*F(1,0) + F(1,1)*F(1,1);
		double detF = F.determinant();

		dEdF = F * shearModulus + FinvT * (-shearModulus + bulkModulus*log(detF));
	}

	//dF/dx is going to be some +/- Xinv terms. The forces on nodes 1,2 can be writen as: dE/dF * XInv', while the force on node 0 is -f1-f2;
	dEdx[1] = V3D(dEdF(0,0) * dXInv(0,0) + dEdF(0,1) * dXInv(0,1), dEdF(1,0) * dXInv(0,0) + dEdF(1,1) * dXInv(0,1), 0) * restShapeArea;
	dEdx[2] = V3D(dEdF(0,0) * dXInv(1,0) + dEdF(0,1) * dXInv(1,1), dEdF(1,0) * dXInv(1,0) + dEdF(1,1) * dXInv(1,1), 0) * restShapeArea;
	dEdx[0] = -dEdx[1]-dEdx[2];
}

void CSTElement2D::computeHessianComponents(const dVector& x, const dVector& X) {
    /*
    H = dfdx = ddEdxdx.
    H = restShapeArea * dPdx(F; dFdx) * transpose(dXInv)
    There are different formula of dPdx in different models. See below.
    dFdx = dDsdx * dXInv
    dDs = [	dx1 - dx0, dx2 - dx0
    dy1 - dy0, dy2 - dy0 ]
    let dx0,dy0,dx1,dy1,dx2,dy2 be 1 respectively (while others keep 0 so that we get dDsd(xk)),
    we can calculate 6 elements of H in each turn ( {l = 0..5}ddEd(xk)d(xl) ), and finally
    fill out whole H matrix. (3*3=9 small 2x2 matrices)
    */
    if (matModel == MM_STVK)
    {
        /*
        dPdx(F; dFdx) = dFdx * (2 * shearModulus * E + bulkModulus * trace(E) * I) +
        F * (2 * shearModulus * dEdx + bulkModulus * trace(dEdx) * I)
        dEdx = 0.5 * (transpose(dFdx) * F + transpose(F) * dFdx)
        */
        computeDeformationGradient(x, X, F);
        Matrix2x2 FT = F.transpose();
        Matrix2x2 E = 0.5 * (FT * F);
        E(0, 0) -= 0.5; E(1, 1) -= 0.5;
        Matrix2x2 I;
        I(0, 0) = I(1, 1) = 1; I(0, 1) = I(1, 0) = 0;
        for (int i = 0;i < 3;++i)
            for (int j = 0;j < 2;++j)
            {
                Matrix2x2 dFdXij;
                dFdXij(0, 0) = dFdXij(0, 1) = dFdXij(1, 0) = dFdXij(1, 1) = 0;
                if (i > 0)
                    dFdXij(j, i - 1) = 1;
                else
                    dFdXij(j, 0) = dFdXij(j, 1) = -1;
                dFdXij = dFdXij * dXInv;
                Matrix2x2 dEdXij = 0.5 * (dFdXij.transpose() * F + FT * dFdXij);
                Matrix2x2 dPdXij = dFdXij * (2.0 * shearModulus * E + bulkModulus * E.trace() * I);
                dPdXij += F * (2.0 * shearModulus * dEdXij + bulkModulus * dEdXij.trace() * I);
                Matrix2x2 dHdXij = restShapeArea * dPdXij * dXInv.transpose();
                for (int ii = 0;ii < 2;++ii)
                    for (int jj = 0;jj < 2;++jj)
                    ddEdxdx[ii + 1][i](jj, j) = dHdXij(jj, ii);
                ddEdxdx[0][i](0, j) = -dHdXij(0, 1) - dHdXij(0, 0);
                ddEdxdx[0][i](1, j) = -dHdXij(1, 1) - dHdXij(1, 0);
            }
        //Logger::consolePrint("%lf %lf\n%lf %lf\n", ddEdxdx[0][1](0, 0), ddEdxdx[0][1](0, 1), ddEdxdx[0][1](1, 0), ddEdxdx[1][1](1, 1));
    }
    else if (matModel == MM_LINEAR_ISOTROPIC)
    {
        //dPdx(F; dFdx) = shearModulus * (dFdx + transpose(dFdx)) + bulkModulus * trace(dFdx) * I
        Matrix2x2 dSdX[6], dFdX, dPdX, dHdX;
        Matrix2x2 MI;
        MI << 1.0, 0.0, 0.0, 1.0;
        dSdX[0] << -1.0, -1.0, 0.0, 0.0;
        dSdX[1] << 1.0, 0.0, 0.0, 0.0;
        dSdX[2] << 0.0, 1.0, 0.0, 0.0;
        dSdX[3] << 0.0, 0.0, -1.0, -1.0;
        dSdX[4] << 0.0, 0.0, 1.0, 0.0;
        dSdX[5] << 0.0, 0.0, 0.0, 1.0;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                ddEdxdx[i][j].setZero();
            }
        }
        for (int i = 0; i < 6; i++) {
            dFdX = dSdX[i] * dXInv;
            dPdX = shearModulus*(dFdX + dFdX.transpose()) + bulkModulus*(dFdX(0, 0) + dFdX(1, 1))*MI;
            dHdX = restShapeArea * dPdX * dXInv.transpose();
            ddEdxdx[i % 3][1](i / 3, 0) = dHdX(0, 0);
            ddEdxdx[i % 3][1](i / 3, 1) = dHdX(1, 0);
            ddEdxdx[i % 3][2](i / 3, 0) = dHdX(0, 1);
            ddEdxdx[i % 3][2](i / 3, 1) = dHdX(1, 1);
            ddEdxdx[i % 3][0](i / 3, 0) = -ddEdxdx[i % 3][2](i / 3, 0) - ddEdxdx[i % 3][1](i / 3, 0);
            ddEdxdx[i % 3][0](i / 3, 1) = -ddEdxdx[i % 3][2](i / 3, 1) - ddEdxdx[i % 3][1](i / 3, 1);
        }
    }
    else if (matModel == MM_NEO_HOOKEAN)
    {
        /*
        dPdx(F; dFdx) = shearModulus * dFdx +
        (shearModulus - bulkModulus * log(det(F))) * FinvT * transpose(dFdx) * FinvT +
        bulkModulus * trace(Finv * dFdx) * FinvT
        Finv = inverse(F)
        FinvT = transpose(Finv)
        */
        computeDeformationGradient(x, X, F);
        Finv = F.inverse();
        FinvT = Finv.transpose();
        Matrix2x2 dF, dP, tmpM, dH;
        const double dDs[6][4] = { { -1,-1,0,0 },{ 0,0,-1,-1 },{ 1,0,0,0 },{ 0,0,1,0 },{ 0,1,0,0 },{ 0,0,0,1 } };
        for (int i = 0; i < 6; ++i)
        {
            for (int x = 0; x < 4; ++x)
                dF(x / 2, x % 2) = dDs[i][x];
            dF = dF * dXInv;
            dP = shearModulus * dF;
            double J = F.determinant();
            dP = dP + (shearModulus - bulkModulus * log(J)) * FinvT * dF.transpose() * FinvT;
            tmpM = Finv * dF;
            dP = dP + bulkModulus * (tmpM(0, 0) + tmpM(1, 1)) * FinvT;
            dH = restShapeArea * dP * dXInv.transpose();
            int row = i / 2, subrow = i % 2;
            ddEdxdx[row][1](subrow, 0) = dH(0, 0); ddEdxdx[row][1](subrow, 1) = dH(1, 0);
            ddEdxdx[row][2](subrow, 0) = dH(0, 1); ddEdxdx[row][2](subrow, 1) = dH(1, 1);
            ddEdxdx[row][0](subrow, 0) = -(dH(0, 0) + dH(0, 1));
            ddEdxdx[row][0](subrow, 1) = -(dH(1, 0) + dH(1, 1));
        }
    }
}