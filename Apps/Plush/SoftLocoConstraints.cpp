#include "SoftLocoConstraints.h"
#include "SoftLocoSolver.h"

SoftLocoConstraints::SoftLocoConstraints(SoftLocoSolver *loco){
	this->loco = loco;

	const int &Z = loco->Z;
	const int  S = loco->S();
	const int  T = loco->T();
	const int ZS = loco->ZS();

	l.setZero(ZS);
	u.setZero(ZS);

	for (int z = 0; z < Z; ++z) {
		for (int s = 0; s < S; ++s) {
			int i = z*S + s;

			double alphaz = loco->mesh->tendons[s % T]->get_alphaz();


			if (s < T) {
				if (z == 0 || z == loco->Z - 1) {
					l[i] = -0.001;
					u[i] = 0.001;
				} else {
					l[i] = -.5*alphaz;
					u[i] = .5*alphaz;
				}
			} else {
				if (z == 0 || z == loco->Z - 1) {
					l[i] = -0.001;
					u[i] = 0.001;
				} else {
					l[i] = -100.;
					u[i] = 100.;
				} 
			}

		} 
	}
}
