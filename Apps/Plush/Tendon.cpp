#include "Tendon.h"

#include <GUILib/GLUtils.h>
#include "SimulationMesh.h"

Tendon::Tendon(SimulationMesh* simMesh, vector<Point *> waypoints) : SimMeshElement(simMesh) {
	this->waypoints = waypoints; // NOTE: Call me first.
	// -- NOTE: Convention followed by { D1, D2, dEdx, ddEdxdx }.
	for (auto &waypoint : waypoints) {
		for (auto &node : waypoint->nodes) {
			unique_push_back(this->IC, node->nodeIndex);
		}
	}
	std::sort(IC.begin(), IC.end());
	// --
	this->c = simMesh->c;
	this->eps = simMesh->eps;
	// TODO: Construct both models, choose between them on the fly with a bool.

	this->quadratic_E = new Quadratic(&this->c);
	this->zcq_E = new ZeroCubicQuadratic(&this->c, &this->eps, V3D(), false, false);
	// dynamic_cast<ZeroCubicQuadratic *>(zcq_E)->test_self();

	// -- Set up shell variables. 
	for (const auto &i : IC) {
		this->D1[i] = V3D();
		this->dEdx[i] = V3D(); 
	}
	// --
	// NOTE: This is likely excessive (will have a lot of 0 blocks).
	for (const auto &i : IC) {
		for (const auto &j : IC) {
			this->D2[std::make_pair(i, j)] = MatrixDxD(); // @
			this->ddEdxdx[std::make_pair(i, j)] = MatrixDxD(); // @
		}
	}
}

Tendon::~Tendon(){ }

void Tendon::draw(const dVector &x, const dVector &balphac) {

	glPushAttrib(GL_ALL_ATTRIB_BITS); {
		glLineWidth(5.);
		glPointSize(7.); 

		P3D COLOR = HENN1NK;
		if (balphac.size() != 0) {
			const P3D POS_COLOR = (SPEC_COLOR == BLACK) ? ORCHID : SPEC_COLOR;
			const P3D NEG_COLOR = (SPEC_COLOR == BLACK) ? RATIONALITY : SPEC_COLOR;
			const P3D ZERO_COLOR = (SPEC_COLOR == BLACK) ? WHITE : SPEC_COLOR;
			const double POS_ALPHAC_SATURATION = .1;
			const double NEG_ALPHAC_SATURATION = .1;
			// --
			// double Gamma = get_Gamma(x, balphac);
			double alphac = get_alphac(balphac);
			double f;
			P3D BASE_COLOR;
			// --
			if (alphac > 0) {
				f = alphac / POS_ALPHAC_SATURATION;
				BASE_COLOR = POS_COLOR;
			} else {
				f = abs(alphac / NEG_ALPHAC_SATURATION);
				BASE_COLOR = NEG_COLOR;
			}
			COLOR = color_swirl(f, ZERO_COLOR, BASE_COLOR);
		}
		// --
		set_color(COLOR);
		glBegin(GL_LINE_STRIP); {
			for (auto &waypoint : waypoints) {
				glP3Dz(waypoint->getCoordinates(x), 2);
			}
		} glEnd();

		set_color(color_shade(COLOR));
		glBegin(GL_POINTS); {
			for (auto &waypoint : waypoints) {
				glP3Dz(waypoint->getCoordinates(x), 3);
			}
		} glEnd();

	} glPopAttrib();

}
 
ScalarFunction *Tendon::tendon_energy_model() {
	return (simMesh->UNILATERAL_TENDONS) ? zcq_E : quadratic_E;
}

////////////////////////////////////////////////////////////////////////////////
// core suite //////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////// 

double Tendon::get_L(const dVector &y) {
	double L = 0.;
	for (size_t k = 0; k <= waypoints.size() - 2; ++k) {
		P3D xtk = waypoints[k]->getCoordinates(y);
		P3D xtkp1 = waypoints[k + 1]->getCoordinates(y);
		V3D sk = xtk - xtkp1;
		L += (sk).norm();
	}
	return L;
}

double Tendon::get_alpha(const dVector &balphac) {
	return get_alphaz() - get_alphac(balphac);
}

double Tendon::get_alphaz() {
	return simMesh->balphaz[tendonIndex];
}

double Tendon::get_alphac(const dVector &balphac) {
	return balphac[tendonIndex];
}

void Tendon::set_alphac(const double &alphac) {
	simMesh->balphac[tendonIndex] = alphac;
}

void Tendon::set_alphac_over_alphaz(const double &alphac_over_alphaz) {
	set_alphac(get_alphaz() * alphac_over_alphaz);
}

// FORNOW: alpha = alpha_0 - alpha_c
//      => alpha_c = alpha_0 - alpha
void Tendon::DEPRECATED_set_alpha(const double &alpha) {
	set_alphac(get_alphaz() - alpha);
}



double Tendon::get_Gamma(const dVector &x, const dVector &balphac) {
	// +: length in mesh is more than rest length => under tension
	// -: rest length is more than length in mesh => slack
	// TODO ($$$)
	return get_L(x) - get_alpha(balphac);
}

double Tendon::get_tau(const dVector &x, const dVector &balphac) {
	return tendon_energy_model()->computeDerivative(get_Gamma(x, balphac));
}

////////////////////////////////////////////////////////////////////////////////
// maff ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int Tendon::D() {
	return simMesh->D();
}
 
MatrixNxM Tendon::MatrixDxD() {
	MatrixNxM m;
	mat_resize_zero(m, D(), D());
	return m;
}

MatrixNxM Tendon::outerDxD(const V3D &u, const V3D &v) {
	if (D() == 2) {
		return outer2x2(u, v);
	} else {
		return outer3x3(u, v); 
	}
}

MatrixNxM Tendon::xxTDxD(const V3D &u) {
	if (D() == 2) {
		return xxT2x2(u);
	} else {
		return xxT3x3(u);
	} 
}

////////////////////////////////////////////////////////////////////////////////
// energy marshaling ///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Tendon::addEnergyGradientTo(const dVector &x, const dVector &balphac, dVector& grad) {
	computeGradientComponents(x, balphac);
	for (const auto &i : IC) {
		for (int j = 0; j < D(); ++j) { // @
			grad[simMesh->nodes[i]->dataStartIndex + j] += dEdx[i][j]; // NOTE: V3D hack
		}
	}
}

void Tendon::addEnergyHessianTo(const dVector& x, const dVector &balphac, std::vector<MTriplet>& hesEntries) {
    computeHessianComponents(x, balphac);
	for (auto &i : IC) {
		for (auto &j : IC) {
			addSparseMatrixDenseBlockToTriplet(hesEntries, simMesh->nodes[i]->dataStartIndex,
														   simMesh->nodes[j]->dataStartIndex,
														   ddEdxdx[std::make_pair(i, j)],
														   true);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
// energy bodies ///////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

double Tendon::getEnergy(const dVector &x, const dVector &balphac) {
	return tendon_energy_model()->computeValue(get_Gamma(x, balphac));
}

double Tendon::get_dEdDelta(const dVector &x, const dVector &balphac) {
	return tendon_energy_model()->computeDerivative(get_Gamma(x, balphac));
}

double Tendon::get_d2Ed2Delta(const dVector &x, const dVector &balphac) {
	return tendon_energy_model()->computeSecondDerivative(get_Gamma(x, balphac));
}

void Tendon::addToD1(int ia, const V3D &v) {
	Point *a = waypoints[ia];
	// -- // D1[i] += v; // (*)
	for (size_t k = 0; k < a->nodes.size(); ++k) {
		Node  *node   = a->nodes[k];
		double weight = a->weights[k];
		D1[node->nodeIndex] += v*weight; // CHECKME
	}

}

void Tendon::addToD2(int ia, int ib, const MatrixNxM &m) {
	Point *a = waypoints[ia];
	Point *b = waypoints[ib];
	// -- // D2[std::make_pair(i, j)] += m; // (**)
	for (size_t k_a = 0; k_a < a->nodes.size(); ++k_a) {
		Node  *node_a   = a->nodes[k_a];
		double weight_a = a->weights[k_a];
		for (size_t k_b = 0; k_b < b->nodes.size(); ++k_b) {
			Node  *node_b   = b->nodes[k_b];
			double weight_b = b->weights[k_b];
			D2[make_pair(node_a->nodeIndex,
						 node_b->nodeIndex)] += m*weight_a*weight_b; // CHECKME // NOTE: I doubt this is right.
		}
	}


}

void Tendon::computeFirstDeltaPartials(const dVector &x, const dVector &balphac) {

	for (const auto &i : IC) {
		D1[i].setZero();
	}

	for (size_t i = 0; i < waypoints.size(); ++i) {
		P3D xi = waypoints[i]->getCoordinates(x);
		// {\hat s}_\alpha
		if (i != waypoints.size() - 1) {
			P3D xip1 = waypoints[i+1]->getCoordinates(x);
			V3D si = xi - xip1;
			V3D si_hat = si.normalized();
			addToD1(i, si_hat);

		}
		// -{\hat s}_{\alpha-1}
		if (i != 0) {
			P3D xim1 = waypoints[i-1]->getCoordinates(x);
			V3D sim1 = xim1 - xi;
			V3D sim1_hat = sim1.normalized();
			addToD1(i, -sim1_hat);
		}
	}
}

void Tendon::computeSecondDeltaPartials(const dVector &x, const dVector &balphac) {

	for (auto &i : IC) {
		for (auto &j : IC) {
			D2[std::make_pair(i, j)].setZero(); // %
		}
	}

	const auto zeta = [this](V3D si) { // @
		double mag_si = si.norm();
		double left_fac = 1. / (mag_si);
		double right_fac = 1. / (mag_si * mag_si * mag_si);
		MatrixNxM zi = MatrixDxD(); // @
		MatrixNxM left = MatrixDxD(); // @
		MatrixNxM right = MatrixDxD(); // @
		left.setIdentity();
		left *= left_fac;
		right = this->xxTDxD(si)*right_fac; // @
		zi = left - right;
		return zi;
	};

	for (size_t i = 0; i < waypoints.size(); ++i) {
		P3D xi = waypoints[i]->getCoordinates(x);
		for (size_t j = 0; j < waypoints.size(); ++j) {
			// \Delta_{t_\alpha t_\alpha}
			if (i == j) {
				// \zeta_\alpha
				if (i != waypoints.size() - 1) {
					P3D xip1 = waypoints[i+1]->getCoordinates(x);
					V3D si = xi - xip1;
					addToD2(i, i, zeta(si));
				}
				// \zeta_{\alpha-1}
				if (i != 0) {
					P3D xim1 = waypoints[i-1]->getCoordinates(x);
					V3D sim1 = xim1 - xi;
					addToD2(i, i, zeta(sim1));
				}
			// \Delta_{t_\alpha t_\beta}
			} else {
				// -\zeta_\alpha
				if (i == j + 1) {
					P3D xj = waypoints[j]->getCoordinates(x);
					P3D xjp1 = waypoints[j+1]->getCoordinates(x);
					V3D sj = xj - xjp1;
					addToD2(i, j, -zeta(sj));
				}
				// -\zeta_\beta
				if (j == i + 1) {
					P3D xi = waypoints[i]->getCoordinates(x);
					P3D xip1 = waypoints[i+1]->getCoordinates(x);
					V3D si = xi - xip1;
					addToD2(i, j, -zeta(si));
				}
			}
		}
	}
}

void Tendon::computeGradientComponents(const dVector &x, const dVector &balphac){
	// zero out
	for (const auto &k : IC) {
		dEdx[k].zero();
	}

	// compute D1
	computeFirstDeltaPartials(x, balphac);
	// marshal
	double dEdDelta = get_dEdDelta(x, balphac);
	for (const auto &i : IC) {
		dEdx[i] = D1[i]*dEdDelta;
	}
}

void Tendon::computeHessianComponents(const dVector &x, const dVector &balphac) {
	// zero out
	for (const auto &i : IC) {
		for (const auto &j : IC) {
			ddEdxdx[std::make_pair(i, j)].setZero();
		}
	}

	// compute D1
	computeFirstDeltaPartials(x, balphac);
	// compute D2
	computeSecondDeltaPartials(x, balphac);
	// marshal
	double dEdDelta = get_dEdDelta(x, balphac);
	double d2Ed2Delta = get_d2Ed2Delta(x, balphac);
	for (const auto &i : IC) {
		for (const auto &j : IC) {
			ddEdxdx[std::make_pair(i, j)] = d2Ed2Delta*outerDxD(D1[i], D1[j]) + dEdDelta*D2[std::make_pair(i, j)]; // @
		}
	}
}
