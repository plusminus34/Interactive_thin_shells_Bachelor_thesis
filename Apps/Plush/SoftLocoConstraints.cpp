#include "SoftLocoConstraints.h"
#include "SoftLocoSolver.h"

SoftLocoConstraints::SoftLocoConstraints(SoftLocoSolver *loco){
	this->loco = loco;

	l.setZero(loco->ZT());
	u.setZero(loco->ZT());
	for (int t = 0; t < loco->T(); ++t) {
		double alphaz = loco->mesh->tendons[t]->get_alphaz();
		for (int z = 0; z < loco->Z; ++z) {
			int i = z*loco->T() + t;
			if (z == 0 || z == loco->Z - 1) {
				l[i] = -0.001;
				u[i] =  0.001;
			} else {
				l[i] = -.5*alphaz;
				u[i] =  .5*alphaz;
			}
		} 
	}
}
