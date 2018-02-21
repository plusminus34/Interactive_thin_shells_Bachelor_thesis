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
	currentPosition = getPosition(x);
	bool PENETRATING = (currentPosition.y() < 0);

	ACTIVE = PENETRATING;
	targetPosition = currentPosition;
	targetPosition.y() = 0.;

	/*
	if (ACTIVE) {
		if (!PENETRATING) {
			ACTIVE = false;
			targetPosition = P3D(0., 0., 0.);
		} 
	} else {
		if (PENETRATING) {
			ACTIVE = true;
			targetPosition = currentPosition;
			targetPosition.y() = 0.;
		} 
	} 
	*/
}

int Contact::D() {
	return simMesh->D();
}

V3D Contact::get_Delta(const dVector &x) {
	// NOTE: dDeltadx = 1
	return node->getCoordinates(x) - targetPosition;
}

double Contact::b_(const double &y) {
	double f_y = abs(ZCQ1->computeDerivative(y));
	// double step = _SSb->g(-y +_eps_b); // ? (*)
	// return f_y * step;
	return f_y;
}

double Contact::get_E(const dVector &x) {
	auto &ds = get_Delta(x); 
	
	double E = 0.;

	for (int i = 0; i < D(); ++i) {
		if (i == 1) { continue; }
		E += b()*QT->computeValue(ds[i]);
	}

	E += ZCQ1->computeValue(ds[1]);
 
	return E;
}

dVector Contact::get_dEdx(const dVector &x) {
	auto &ds = get_Delta(x); 

	dVector dEdx; resize_zero(dEdx, D());

	for (int i = 0; i < D(); ++i) {
		if (i == 1) { continue; }
		dEdx[i] += b()*QT->computeDerivative(ds[i]);
	}

	dEdx[1] += ZCQ1->computeDerivative(ds[1]);

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
		ddEdxdx(i, i) += b()*QT->computeSecondDerivative(ds[i]);
	}

	ddEdxdx(1, 1) += ZCQ1->computeSecondDerivative(ds[1]); // (*) 

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