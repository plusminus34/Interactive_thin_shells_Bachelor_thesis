#include "Contact.h"
// -- 
#include "SimulationMesh.h"
#include "Frame.h"

double Contact::K_x   = 750;
double Contact::K_y   = 750;
double Contact::eps_y = .02;
Quadratic *Contact::QT = new Quadratic(&Contact::K_x);
ZeroCubicQuadratic *Contact::ZCQ1 = new ZeroCubicQuadratic(&Contact::K_y, &Contact::eps_y, V3D(), true, false);

Contact::Contact(Node *node, SimulationMesh *simMesh) : SimMeshElement(simMesh) {
	this->node = node;
}

void Contact::draw(const dVector &x) {
	int Z = 0;

	P3D LINE_COLOR = (ACTIVE) ? ACTIVE_COLOR : INACTIVE_COLOR;
	P3D POINT_COLOR = color_shade(LINE_COLOR);

	glMasterPush(); {
		glLineWidth(4.);
		glPointSize(9.);

		if (ACTIVE) {
			vector<P3D> tmp = { node->getCoordinates(x), targetPosition };

			set_color(POINT_COLOR);
			glBegin(GL_POINTS); { glvecP3Dz(tmp, Z); } glEnd();

			set_color(LINE_COLOR);
			glBegin(GL_LINES); { glvecP3Dz(tmp, Z); } glEnd();
		}
		else {
			set_color(POINT_COLOR);
			glBegin(GL_POINTS); { glP3Dz(node->getCoordinates(x), Z); } glEnd();
		}

	} glMasterPop();

	// -- // FORNOW
	if (D() == 2) {
		glMasterPush(); {
			glLineWidth(4.);
			set_color(ORCHID);
			// --
			glBegin(GL_LINES); {
				glP3D(P3D(-100., 0.));
				glP3D(P3D(100., 0.));
			} glEnd();
		} glMasterPop();
	}
}

P3D Contact::getPosition(const dVector &x) {
	return node->getCoordinates(x);
}
 
void Contact::update(const dVector &x) {
	targetPosition = getPosition(x);
	targetPosition[1] = 0.;
	ACTIVE = true; // (currentPosition.y() < 0);
}

int Contact::D() {
	return simMesh->D();
}

V3D Contact::get_Delta(const dVector &x) {
	// NOTE: dDeltadx = 1
	return getPosition(x) - targetPosition;
}

double Contact::b_(const double &y) {
	double f_y = -ZCQ1->computeDerivative(y);
	if (f_y < 0) { error("[b_] ValueError"); }
	return f_y;
}

double Contact::b_prime_(const double &y) {
	return -ZCQ1->computeSecondDerivative(y);
}

double Contact::get_E(const dVector &x) {
	
	double E = 0.;

	auto &ds = get_Delta(x); 
	for (int i = 0; i < D(); ++i) {
		if (i == 1) { continue; }
		E += b(x)*QT->computeValue(ds[i]);
	}

	E += ZCQ1->computeValue(getPosition(x)[1]); // FORNOW: NOTE: targetPosition[1] <- 0.
 
	return E;
}

dVector Contact::get_dEdx(const dVector &x) {
	auto &ds = get_Delta(x); 

	dVector dEdx; resize_zero(dEdx, D());

	for (int i = 0; i < D(); ++i) {
		if (i == 1) { continue; }
		dEdx[i] += b(x)*QT->computeDerivative(ds[i]);
	}

	dEdx[1] += ZCQ1->computeDerivative(getPosition(x)[1]);

	{
		dVector f_ctc = -dEdx;
		if (f_ctc[1] < 0) {
			error("Normal force is facing down."); 
		}
	}

	return dEdx;
}

MatrixNxM Contact::get_ddEdxdx(const dVector &x) {
	auto &ds = get_Delta(x); 
	
	MatrixNxM ddEdxdx; mat_resize_zero(ddEdxdx, D(), D());

	for (int i = 0; i < D(); ++i) {
		if (i == 1) { continue; }
		ddEdxdx(i, i) += b(x)*QT->computeSecondDerivative(ds[i]);
	}

	ddEdxdx(1, 1) += ZCQ1->computeSecondDerivative(getPosition(x)[1]); // (*) 

	return ddEdxdx; 
}

double Contact::getEnergy(const dVector &x, const dVector &_){
	// update(x);
	// --
	if (!ACTIVE) { return 0.; }
	if (FORCE_INACTIVE_FOR_TESTING_DRAWING) { return 0.; }
	// --
	return get_E(x);
}


void Contact::addEnergyGradientTo(const dVector &x, const dVector &_, dVector &grad) {
	// update(x);
	// --
	if (!ACTIVE) { return; }
	if (FORCE_INACTIVE_FOR_TESTING_DRAWING) { return; }
 
	grad.segment(node->dataStartIndex, D()) += get_dEdx(x);
}

void Contact::addEnergyHessianTo(const dVector &x, const dVector &_, std::vector<MTriplet> &hesEntries){
	// update(x);
	// --
	if (!ACTIVE) { return; }
	if (FORCE_INACTIVE_FOR_TESTING_DRAWING) { return; }

	addSparseMatrixDenseBlockToTriplet(hesEntries, node->dataStartIndex, node->dataStartIndex, get_ddEdxdx(x), true);
}
