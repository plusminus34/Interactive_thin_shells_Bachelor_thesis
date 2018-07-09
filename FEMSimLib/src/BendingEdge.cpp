#include <FEMSimLib/BendingEdge.h>
#include <GUILib/GLUtils.h>
#include <FEMSimLib/SimulationMesh.h>

BendingEdge::BendingEdge(SimulationMesh* simMesh, Node* n1, Node* n2, Node* n3, Node* n4) : SimMeshElement(simMesh) {
	this->n[0] = n1;
	this->n[1] = n2;
	this->n[2] = n3;
	this->n[3] = n4;

	setRestShapeFromCurrentConfiguration();
}

BendingEdge::~BendingEdge(){}

void BendingEdge::setRestShapeFromCurrentConfiguration() {
	P3D x0 = n[0]->getWorldPosition();
	P3D x1 = n[1]->getWorldPosition();
	P3D x2 = n[2]->getWorldPosition();
	P3D x3 = n[3]->getWorldPosition();

	P3D e0 = x1 - x0;
	P3D n1 = e0.cross(x2 - x0);
	P3D n2 = (x3 - x0).cross(e0);
	int sign = SGN(n1.cross(n2).dot(e0));
	double l_n1 = n1.norm();
	double l_n2 = n2.norm();

	restAngle = acos(n1.dot(n2) / (l_n1*l_n2))*sign;
	restEdgeLength = e0.norm();
	restArea = 0.5 * (l_n1 + l_n2);
}

double BendingEdge::getMass() {
	return 0;
}

double BendingEdge::getAngle(const dVector& x) {
	P3D x0 = n[0]->getCoordinates(x);
	P3D x1 = n[1]->getCoordinates(x);
	P3D x2 = n[2]->getCoordinates(x);
	P3D x3 = n[3]->getCoordinates(x);

	P3D e0 = x1 - x0;
	P3D n1 = e0.cross(x2 - x0);
	P3D n2 = (x3 - x0).cross(e0);
	int sign = SGN(n1.cross(n2).dot(e0));
	double ratio = std::max(std::min(n1.dot(n2) / (n1.norm()*n2.norm()), 1.0), -1.0);
	return acos(ratio)*sign;
}

double BendingEdge::getEnergy(const dVector& x, const dVector& X) {
	double d_angle = getAngle(x) - restAngle;
	/* alternative energy: 
	k*3*restEdgeLength*restEdgeLength/restArea  * (phi(angle) - phi(restAngle))^2
	phi(alpha) = 2 * tan(alpha/2)	and	psi(alpha) = tan(alpha/pow(2,n))	s.t. phi = 2^n psi
	*/
	return k * 3 * restEdgeLength*restEdgeLength * d_angle*d_angle / restArea;
}

void BendingEdge::addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) {
	//define various values needed
	P3D x0 = n[0]->getCoordinates(x);
	P3D x1 = n[1]->getCoordinates(x);
	P3D x2 = n[2]->getCoordinates(x);
	P3D x3 = n[3]->getCoordinates(x);
	double d_angle = getAngle(x) - restAngle;
	P3D e0 = x1 - x0;
	P3D e1 = x2 - x0;
	P3D e2 = x3 - x0;
	P3D e3 = x2 - x1;
	P3D e4 = x3 - x1;
	P3D n1 = e0.cross(e1);
	P3D n2 = e2.cross(e0);
	double l_e0 = e0.norm(); e0 /= l_e0;
	double l_e1 = e1.norm(); e1 /= l_e1; 
	double l_e2 = e2.norm(); e2 /= l_e2;
	double l_e3 = e3.norm(); e3 /= l_e3;
	double l_e4 = e4.norm(); e4 /= l_e4;
	double l_n1 = n1.norm(); n1 /= l_n1;
	double l_n2 = n2.norm(); n2 /= l_n2;
	double angle_1 = acos(e0.dot(e1));
	double angle_2 = acos(e2.dot(e0));
	double angle_3 = acos(e3.dot(-e0));
	double angle_4 = acos((-e0).dot(e4));
	double h_1 = l_n1 / l_e1;
	double h_2 = l_n2 / l_e2;
	double h_3 = l_n1 / l_e3;
	double h_4 = l_n2 / l_e4;
	double h_01 = l_n1 / l_e0;
	double h_02 = l_n2 / l_e0;

	// constant factor depending only on rest shape
	double K = k * 6 * restEdgeLength*restEdgeLength / restArea;
	// combine with factor depending on current angle
	double zeta = K * d_angle;// zeta changes when using alternative energy, see Hessian

	P3D component;
	// dE/dx0
	component = (n1*cos(angle_3) / h_3 + n2 * cos(angle_4) / h_4)*zeta;
	for (int i = 0;i < 3;++i)grad[n[0]->dataStartIndex + i] += component[i];
	// dE/dx1
	component = (n1*cos(angle_1) / h_1 + n2 * cos(angle_2) / h_2)*zeta;
	for (int i = 0;i < 3;++i)grad[n[1]->dataStartIndex + i] += component[i];
	// dE/dx2
	component = n1 * (-zeta) / h_01;
	for (int i = 0;i < 3;++i)grad[n[2]->dataStartIndex + i] += component[i];
	// dE/dx3
	component = n2 * (-zeta) / h_02;
	for (int i = 0;i < 3;++i)grad[n[3]->dataStartIndex + i] += component[i];
}

void BendingEdge::addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries) {
	// constant factors
	double d_angle = getAngle(x) - restAngle;
	double K = k * 6 * restEdgeLength*restEdgeLength / restArea;
	double zeta = K * d_angle;// for alternative enrgy: zeta = K*(phi-rest_phi)*(1+psi*psi)
	double xi = K;// for alternative enrgy: xi = K*(1+psi*psi)*(2*(psi-rest_psi)*psi + (1+psi*psi))

	//start copied code from gradient
	Vector3d x0 = n[0]->getCoordinates(x);
	Vector3d x1 = n[1]->getCoordinates(x);
	Vector3d x2 = n[2]->getCoordinates(x);
	Vector3d x3 = n[3]->getCoordinates(x);
	Vector3d e0 = x1 - x0;
	Vector3d e1 = x2 - x0;
	Vector3d e2 = x3 - x0;
	Vector3d e3 = x2 - x1;
	Vector3d e4 = x3 - x1;
	Vector3d n1 = e0.cross(e1);
	Vector3d n2 = e2.cross(e0);
	double l_e0 = e0.norm(); e0 /= l_e0;
	double l_e1 = e1.norm(); e1 /= l_e1;
	double l_e2 = e2.norm(); e2 /= l_e2;
	double l_e3 = e3.norm(); e3 /= l_e3;
	double l_e4 = e4.norm(); e4 /= l_e4;
	double l_n1 = n1.norm(); n1 /= l_n1;
	double l_n2 = n2.norm(); n2 /= l_n2;
	double angle_1 = acos(e0.dot(e1));
	double angle_2 = acos(e2.dot(e0));
	double angle_3 = acos(e3.dot(-e0));
	double angle_4 = acos((-e0).dot(e4));
	double h_1 = l_n1 / l_e1;
	double h_2 = l_n2 / l_e2;
	double h_3 = l_n1 / l_e3;
	double h_4 = l_n2 / l_e4;
	double h_01 = l_n1 / l_e0;
	double h_02 = l_n2 / l_e0;
	//end copied code from gradient

	//Gradient of angle
	Vector3d grad_angle[4];
	grad_angle[0] = n1 * cos(angle_3) / h_3 + n2 * cos(angle_4) / h_4;
	grad_angle[1] = n1 * cos(angle_1) / h_1 + n2 * cos(angle_2) / h_2;
	grad_angle[2] = -n1 / h_01;
	grad_angle[3] = -n2 / h_02;

	// vectors m
	Vector3d m1 = -e1.cross(n1);
	Vector3d m2 = e2.cross(n2);
	Vector3d m3 = e3.cross(n1);
	Vector3d m4 = -e4.cross(n2);
	Vector3d m01 = e0.cross(n1);
	Vector3d m02 = -e0.cross(n2);

	//Hessian of angle
	Matrix3x3 H_angle[4][4];
	//Eigen is making v1 * v2.T harder than it should be
	Eigen::RowVector3d n1_t = n1.transpose();
	Eigen::RowVector3d n2_t = n2.transpose();
	Eigen::RowVector3d m1_t = m1.transpose();
	Eigen::RowVector3d m2_t = m2.transpose();
	Eigen::RowVector3d m3_t = m3.transpose();
	Eigen::RowVector3d m4_t = m4.transpose();
	Eigen::RowVector3d m01_t = m01.transpose();
	Eigen::RowVector3d m02_t = m02.transpose();
	Matrix3x3 B1 = n1 * m01_t / (l_e0*l_e0);
	Matrix3x3 B2 = n2 * m02_t / (l_e0*l_e0);

	H_angle[0][0] = cos(angle_3) / (h_3*h_3) * (m3 * n1_t + n1 * m3_t) - B1
		+ cos(angle_4) / (h_4*h_4) * (m4 * n2_t + n2 * m4_t) - B2;
	H_angle[0][1] = cos(angle_3) / (h_1*h_3) * m1*n1_t + cos(angle_1) / (h_1*h_3)*n1*m3_t + B1
		+ cos(angle_4) / (h_2*h_4)*m2*n2_t + cos(angle_2) / (h_2*h_4)*n2*m4_t + B2;
	H_angle[0][2] = cos(angle_3) / (h_3*h_01)*m01*n1_t - n1 * m3_t / (h_01*h_3);
	H_angle[0][3] = cos(angle_4) / (h_4*h_02)*m02*n2_t - n2 * m4_t / (h_02*h_4);
	H_angle[1][1] = cos(angle_1) / (h_1*h_1)*(m1*n1_t + n1 * m1_t) - B1
		+ cos(angle_2) / (h_2*h_2)*(m2*n2_t + n2 * m2_t) - B2;
	H_angle[1][2] = cos(angle_1) / (h_1*h_01)*m01*n1_t - n1 * m1_t / (h_01*h_1);
	H_angle[1][3] = cos(angle_2) / (h_2*h_02)*m02*n2_t - n2 * m2_t / (h_02*h_2);
	H_angle[2][2] = -(n1*m01_t + m01 * n1_t) / (h_01*h_01);
	H_angle[2][3].setZero();
	H_angle[3][3] = -(n2*m02_t + m02 * n2_t) / (h_02*h_02);
	for (int i = 1; i < 4;++i)
		for (int j = i - 1;j >= 0;--j)
			H_angle[i][j] = H_angle[j][i].transpose();

	//H(E) =  zeta * H(angle) + xi * grad(angle).transpose * grad(angle)
	Eigen::RowVector3d grad_angle_t[4];
	for (int i = 0;i < 4;++i) grad_angle_t[i] = grad_angle[i].transpose();
	Matrix3x3 H[4][4];
	for (int i = 0;i < 4;++i)
		for (int j = 0;j < 4;++j)
			H[i][j] = zeta * H_angle[i][j] + xi * grad_angle[i] * grad_angle_t[j];

	//Finally, convert to triplets
	for(int i=0;i<4;++i)
		for(int j=0;j<4;++j)
			for (int ii = 0; ii < 3; ++ii) {
				int global_j = n[i]->dataStartIndex + ii;
				for (int jj = 0; jj < 3; ++jj) {
					int global_i = n[j]->dataStartIndex + jj;
					if (global_i >= global_j)
						hesEntries.push_back(MTriplet(global_i, global_j, H[i][j](ii, jj)));
				}
			}
}

void BendingEdge::draw(const dVector& x) {
	/*
	P3D x0 = n[0]->getCoordinates(x);
	P3D x1 = n[1]->getCoordinates(x);
	P3D x2 = n[2]->getCoordinates(x);
	P3D x3 = n[3]->getCoordinates(x);

	P3D pa = (x0 + x1 + x2) / 3;
	P3D pb = (x0 + x1 + x3) / 3;

	// draw a line between the midpoints of the triangles
	glBegin(GL_LINES);
	glVertex3d(pa[0], pa[1], pa[2]);
	glVertex3d(pb[0], pb[1], pb[2]);
	glEnd();
	*/
}

void BendingEdge::drawRestConfiguration(const dVector& X) {
}

