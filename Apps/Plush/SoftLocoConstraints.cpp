#include "SoftLocoConstraints.h"
#include "SoftLocoSolver.h"

SoftLocoConstraints::SoftLocoConstraints(SoftLocoSolver *loco){
	this->loco = loco;


	l.setZero(ZS()); l.fill(-INFINITY);
	u.setZero(ZS()); u.fill( INFINITY);
	d.setZero(KT()); d.fill(-INFINITY);
	f.setZero(KT()); f.fill( INFINITY);

	// TODO: Make static member like Contact::K_X, etc.
	double facaz_LOWER_BOUND = -.25;
	double facaz_UPPER_BOUND =  .25;

	// -- primary bounds
	for (int t = 0; t < T(); ++t) {
		double alphaz = loco->mesh->tendons[t]->get_alphaz();
		for (int z = 0; z < Z(); ++z) {
			int i = z*S() + t; 
			// --
			l[i]       = facaz_LOWER_BOUND*alphaz; u[i]       = facaz_UPPER_BOUND*alphaz; // -- y bounds
			l[T() + i] = -INFINITY;                u[T() + i] = INFINITY;                 // -- m bounds
		}

		for (int k = 0; k < K(); ++k) {
			int j = k*T() + t; 
			d[j] = facaz_LOWER_BOUND*alphaz; f[j] = facaz_UPPER_BOUND*alphaz; // -- u(y, m) bounds
		}
	}

	// -- secondary bounds (set ends to zero)
	double SOFT_ZERO = .001;
	for (int t = 0; t < T(); ++t) {
		for (auto &z : { 0, Z() - 1 }) {
			int i = z*S() + t; 
			int j = z*T() + t; 
			l[i]       = -SOFT_ZERO; u[i]       =  SOFT_ZERO;
			l[T() + i] = -SOFT_ZERO; u[T() + i] =  SOFT_ZERO;
		}
	} 
}

const dVector &SoftLocoConstraints::getInequalityConstraintValues(const dVector &ymS) {
	auto ymJ = loco->unstack_Traj(ymS, Z()); // TODO: Strip this into function of this class.
	auto uJ = loco->uJ_of_ymJ(ymJ); // FORNOW
	// --
	ineqConstraintVals = stack_vec_dVector(uJ);
	return ineqConstraintVals;
}

void SoftLocoConstraints::checkConstraints(const vector<dVector> &ymJ) {
	dVector p = stack_vec_dVector(ymJ); 
	dVector Cp = getInequalityConstraintValues(p);
	bool bounds_pass = true;
	bool inequalities_pass = true;
	for (int i = 0; i < ZS(); ++i) {
		if (l[i] > p[i]) { bounds_pass = false; }
		if (p[i] > u[i]) { bounds_pass = false; }
	}
	for (int i = 0; i < KT(); ++i) {
		if ( d[i] > Cp[i]) { inequalities_pass = false; }
		if (Cp[i] >  f[i]) { inequalities_pass = false; }
	}
	cout << "      bounds pass: " <<       bounds_pass << endl;
	cout << "inequalities pass: " << inequalities_pass << endl;
}

int SoftLocoConstraints::K()  { return loco->K; }
int SoftLocoConstraints::Z()  { return loco->Z; }
int SoftLocoConstraints::S()  { return loco->S(); }
int SoftLocoConstraints::T()  { return loco->T(); }
int SoftLocoConstraints::ZS() { return loco->ZS(); }
int SoftLocoConstraints::KT() { return K()*T(); }
