#include "SoftLocoConstraints.h"
#include "SoftLocoSolver.h"

SoftLocoConstraints::SoftLocoConstraints(SoftLocoSolver *loco){
	this->loco = loco;


	l.setZero(ZS());
	u.setZero(ZS());
	d.setZero(KT());
	f.setZero(KT());

	// TODO: Rewrite as two pieces, first sets alphaz bounds, second pins start and end to 0
	for (int z = 0; z < Z(); ++z) {
		for (int s = 0; s < S(); ++s) {
			int i = z*S() + s;

			double alphaz = loco->mesh->tendons[s % T()]->get_alphaz();
 
			if (s < T()) {
				if (z == 0 || z == Z() - 1) {
					l[i] = -0.001;
					u[i] = 0.001;
				} else {
					l[i] = -.2*alphaz;
					u[i] = .2*alphaz;
				}
			} else {
				if (z == 0 || z == Z() - 1) {
					l[i] = -0.001;
					u[i] = 0.001;
				} else {
					l[i] = -INFINITY;
					u[i] =  INFINITY;
				} 
			}

		} 
	}

	for (int k = 0; k < K(); ++k) {
		for (int t = 0; t < T(); ++t) {
			int i = k*T() + t;
			double alphaz = loco->mesh->tendons[t]->get_alphaz();
			d[i] = -.2*alphaz;
			f[i] =  .2*alphaz;
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
