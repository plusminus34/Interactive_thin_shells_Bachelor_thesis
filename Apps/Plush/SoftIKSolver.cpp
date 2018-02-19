#include "SoftIKSolver.h"
// --
#include <PlushHelpers/helpers_star.h>
#include <Eigen/SparseCholesky>
#include "Tendon.h"

SoftIKSolver::SoftIKSolver(SimulationMesh *mesh) {
	this->mesh = mesh; 
	x_0 = mesh->x;
	v_0 = mesh->v;
	// --
	resize_zero(alphac_curr, T());
	x_curr = x_of_alphac(alphac_curr); 
	// --
	construct_alphac_barrierFuncs(); 
	COMp = mesh->get_COM(mesh->X); 
	resize_zero(Z_01_bool_spoof, N());
}

void SoftIKSolver::draw() {
	if (SPEC_FREESTYLE) {
		vector<P3D> tmp_x;
		vector<P3D> tmp_x_prime;
		for (int i = 0; i < N(); ++i) {
			if (Z_01_bool_spoof[i] > .5) {
				auto &node = mesh->nodes[i];
				tmp_x.push_back(node->getCurrentPosition());
				tmp_x_prime.push_back(node->getTargetPosition());
			}
		} 
		glMasterPush(); {
			glLineWidth(3);
			glBegin(GL_LINES); {
				for (size_t i = 0; i < tmp_x.size(); ++i) {
					set_color(ORCHID);
					glP3D(tmp_x[i]);
					set_color(HENN1NK);
					glP3Dz(tmp_x_prime[i], 9);
				}
			} glEnd();
		} glMasterPop();
	}
	
	if (SPEC_COM) {
		glMasterPush();  {
			glPointSize(10);
			glLineWidth(5);
			for (auto &gl_begin : { GL_LINES, GL_POINTS }) {
				glBegin(gl_begin); {
					set_color(PUMPKIN);
					glP3Dz(mesh->get_COM(mesh->x), 9);
					set_color(RATIONALITY);
					glP3Dz(COMp, 9);
				} glEnd();
			}
		} glMasterPop(); 
	} 
}
 
void SoftIKSolver::step() {
	mesh->update_contacts(x_0); // FORNOW
	if (PROJECT) { project(); }
	for (int _ = 0; _ < NUM_ITERS_PER_STEP; ++_) {
		iterate();
	}
}

void SoftIKSolver::iterate() {
	alphac_curr = alphac_next(alphac_curr, x_curr);
	x_curr  = x_of_alphac(alphac_curr); 
}

void SoftIKSolver::project() { 
	dVector alphac_proj = alphac_curr;
	while (true) {
		bool PROJECTED = false;

		dVector x_proj = x_of_alphac(alphac_proj);

		for (int i = 0; i < T(); ++i) { 
			double Gamma_i;
			Gamma_i = mesh->tendons[i]->get_Gamma(x_proj, alphac_proj);

			if (Gamma_i < 0) {
				PROJECTED = true;
				alphac_proj[i] += abs(Gamma_i) + .001;
			}
		}

		if (!PROJECTED) {
			break;
		}
	} 
	alphac_curr = alphac_proj;
	x_curr  = x_of_alphac(alphac_curr);
}

dVector SoftIKSolver::alphac_next(const dVector &alphac, const dVector &x) {
	dVector dOdalphac = calculate_dOdalphac(alphac, x);
	double gamma = calculate_gamma(alphac, dOdalphac);
	return alphac - gamma * dOdalphac; 
}

dVector SoftIKSolver::calculate_dOdalphac(const dVector &alphac, const dVector &x) {
	// dVector dQdalphac = calculate_dQdalphac(alphac, x, true);
	dVector dQdalphac = calculate_dxdalphac(alphac, x) * calculate_dQdx(alphac, x);
	dVector dRdalphac = calculate_dRdalphac(alphac, x);
	return dQdalphac + dRdalphac;
}

double SoftIKSolver::calculate_gamma(const dVector &alphac, const dVector &dOdalphac) {
	double gamma_0 = 1.;
	int maxLineSearchIterations = 10;

	dVector alphac_0 = alphac;
	double O_0 = calculate_O(alphac_0); 
	double gamma_k = gamma_0;
	for (int j = 0; j < maxLineSearchIterations; j++) { 
		dVector alphac_k = alphac_0 - gamma_k * dOdalphac; 
		double O_k = calculate_O(alphac_k);

		if (!isfinite(O_k)) {
			error("non-finite O_k detected.");
			O_k = O_0 + 1.0;
		}

		if (O_k > O_0) {
			gamma_k /= 2.0;
		} else {
			// success
			if (VERBOSE && LINEAR_APPROX) { cout << "O_approx  " << O_k << endl; }
			return gamma_k;
		}
	}

	// error("Line search failed.");
	return gamma_k; 
}

// -- //

dVector SoftIKSolver::x_of_alphac(const dVector &alphac) { // FORNOW
	auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(timeStep, x_0, v_0, alphac) : mesh->solve_statics(x_0, alphac);
	return xv.first;
}

// -- //

double SoftIKSolver::calculate_O(const dVector &alphac) {
	double Q = (LINEAR_APPROX) ? calculate_Q_approx(alphac, false) : calculate_Q_formal(alphac, false);
	return Q + calculate_R(alphac, false);
}
 
double SoftIKSolver::calculate_Q(const dVector &x) {
	double Q = 0.;

	if (SPEC_FREESTYLE) {
		dVector Deltax_ = mesh->x_prime - x;
		Q += .5*Deltax_.transpose()*Z().asDiagonal()*Deltax_;
	}

	if (SPEC_COM) {
		// (*) D()-invariant hack
		V3D DeltaCOM_ = V3D(COMp, mesh->get_COM(x));
		Q += .5*DeltaCOM_.squaredNorm();
	}

	return Q;
}
 
double SoftIKSolver::calculate_Q_formal(const dVector &alphac, bool verbose) {
	dVector x = x_of_alphac(alphac);
	double Q_formal  = calculate_Q(x);
	if (verbose) { cout << "Q_formal  " << Q_formal << endl; }
	return Q_formal;
}

double SoftIKSolver::calculate_Q_approx(const dVector &alphac, bool verbose) { 
	dVector dalphac = alphac - alphac_curr;
	dVector x_approx = x_curr + dxdalphac.transpose() * dalphac; 
	double Q_approx = calculate_Q(x_approx); 
	if (verbose) { cout << "Q_approx  " << Q_approx << endl; }
	return Q_approx;
}

double SoftIKSolver::calculate_R(const dVector &alphac, bool verbose) {

	double ret = 0.;
	double tmp = 0.;

	auto report = [verbose, &tmp, &ret](string name) {
		if (verbose) {
			cout << name << ret - tmp << endl;
			tmp = ret;
		}
	};

	{
		for (int i = 0; i < T(); ++i) {
			ret += alphac_barrierFuncs[i]->computeValue(alphac[i]);
		}
		report("R_barrier  ");
	}

	if (REGULARIZE_alphac) {
		for (int i = 0; i < T(); ++i) {
			ret += alphac_regFunc->computeValue(alphac[i]);
		}
		report("R_alphac   ");
	}

	if (REGULARIZE_honey) {
		dVector x = x_of_alphac(alphac);
		for (int i = 0; i < D()*N(); ++i) {
			ret += honey_regFunc->computeValue(x[i] - x_honey[i]);
		} 
		report("R_honey   ");
	}

	return ret; 
}

dVector SoftIKSolver::calculate_dQdx(const dVector &alphac, const dVector &x) { 
	dVector dQdx; resize_zero(dQdx, D()*N());

	if (SPEC_FREESTYLE) {
		dQdx += Z().asDiagonal()*(x - mesh->x_prime);
	}

	if (SPEC_COM) {
		V3D DeltaCOM_ = V3D(COMp, mesh->get_COM(x));
		dVector DeltaCOM; resize_zero(DeltaCOM, D());
		for (int i = 0; i < D(); ++i) { DeltaCOM[i] = DeltaCOM_[i]; }
		// --
		dVector dQdCOM = DeltaCOM; 
		dQdx += mesh->get_dCOMdx().transpose() * dQdCOM;
	}

	return dQdx;
}

MatrixNxM SoftIKSolver::calculate_dxdalphac(const dVector &alphac, const dVector &x) {
	MatrixNxM dxdtau; // H^{-1} A
	{
		// X H = A_T
		// H_T X_T = A
		SparseMatrix A_s = calculate_A(x);
		SparseMatrix H_T_s = calculate_H(x, alphac).transpose();
		MatrixNxM X_T;
		mat_resize_zero(X_T, H_T_s.cols(), A_s.cols());

		// Decompose LHS matrix;
		Eigen::SimplicialLDLT<SparseMatrix> solver;
		solver.compute(H_T_s);
		if (solver.info() != Eigen::Success) {
			error("Eigen::SimplicialLDLT decomposition failed.");
		}

		// Solve by column.
		dVector x_j, a_j;
		for (int j = 0; j < A_s.cols(); ++j) {
			a_j = A_s.col(j);
			x_j = solver.solve(a_j);
			if (solver.info() != Eigen::Success) {
				error("Eigen::SimplicialLDLT solve failed.");
			}
			X_T.col(j) = x_j;
		}
		dxdtau = X_T.transpose();
	}

	dVector dtaudGamma_diag;
	{
		resize_zero(dtaudGamma_diag, T());
		for (int j = 0; j < T(); ++j) {
			auto &tendon = mesh->tendons[j];
			auto *E_j = tendon->tendon_energy_model();
			double Gamma_j = tendon->get_Gamma(x, alphac);
			dtaudGamma_diag[j] = E_j->computeSecondDerivative(Gamma_j);
		}
	}
 
	dtaudalphac  = dtaudGamma_diag.asDiagonal(); // NOTE: dGammadalphac = I
	dxdalphac    = dtaudalphac     * dxdtau;

	return dxdalphac;
}

dVector SoftIKSolver::calculate_dRdalphac(const dVector &alphac, const dVector &x) { 
	dVector dRdalphac; resize_zero(dRdalphac, T());

	{
		dVector dalphacBarrierdalphac; resize_zero(dalphacBarrierdalphac, T());
		for (int i = 0; i < T(); ++i) {
			dalphacBarrierdalphac[i] += alphac_barrierFuncs[i]->computeDerivative(alphac[i]);
		}
		dRdalphac += dalphacBarrierdalphac;
	}

	if (REGULARIZE_alphac) { 
		dVector dalphacRegdalphac; resize_zero(dalphacRegdalphac, T());
		{
			for (int i = 0; i < T(); ++i) {
				dalphacRegdalphac[i] += alphac_regFunc->computeDerivative(alphac[i]);
			}
		} 
		dRdalphac += dalphacRegdalphac;
	}
 
	if (REGULARIZE_honey) {
		dVector dhoneyRegdx; resize_zero(dhoneyRegdx, D()*N());
		{
			for (int i = 0; i < D()*N(); ++i) {
				dhoneyRegdx[i] += honey_regFunc->computeValue(x[i] - x_honey[i]);
			}
		}
		dRdalphac += dxdalphac * dhoneyRegdx;
	}

	return dRdalphac; 
}

dVector SoftIKSolver::Z() { 
	vector<bool> Z_bool;
	for (int i = 0; i < Z_01_bool_spoof.size(); ++i) {
		Z_bool.push_back((Z_01_bool_spoof[i] > .5));
	}

	dVector ret;
	resize_zero(ret, D()*N());

	// -- // Special Case: Fast return to rest pose.
	if (all_false(Z_bool)) {
		resize_fill(ret, D()*N(), Z_SPECIAL);
		return ret;
	}

	// -- // Typical Case
	for (int i = 0; i < N(); ++i) {
		double Z_i = (Z_bool[i]) ? Z_ON : Z_OFF;
		for (int d = 0; d < D(); ++d) {
			ret[D()*i + d] = Z_i;
		}
	}
	return ret;
}

void SoftIKSolver::toggle_Z(Node *node) {
	if (node == nullptr) { error("nullptr was passed as Node *"); }
	// --
	Z_01_bool_spoof[node->nodeIndex] *= -1;
	Z_01_bool_spoof[node->nodeIndex] +=  1;
}

 
SparseMatrix SoftIKSolver::calculate_A(const dVector &x) {
	SparseMatrix A;
	A.resize(D()*N(), T());
	A.setZero();
	vector<MTriplet> triplets;

	dVector Dj;
	resize_zero(Dj, A.rows());
	for (int j = 0; j < A.cols(); ++j) {
		Dj.setZero();
		// --
		Tendon *tendon = mesh->tendons[j];
		for (size_t u = 0; u < tendon->waypoints.size() - 1; ++u) {
			int v = u + 1;
			// --
			Point *p_u = tendon->waypoints[u];
			Point *p_v = tendon->waypoints[v];
			// --
			P3D s_u = p_u->getCoordinates(x);
			P3D s_v = p_v->getCoordinates(x);
			V3D e_hat(s_u, s_v);
			e_hat.normalize();
			// --
			// dVector Cab;
			// resize_zero(Cab, A.rows());
			{
				{
					vector<Node *> n_u = p_u->nodes;
					vector<double> w_u = p_u->weights;
					// --
					for (size_t i_u = 0; i_u < p_u->nodes.size(); ++i_u) {
						int    n = n_u[i_u]->nodeIndex;
						double w = w_u[i_u];
						// --
						for (int d = 0; d < D(); ++d) {
							// Cab[D()*n + d] += w*e_hat[d];
							triplets.push_back(MTriplet(D()*n + d, j, w*e_hat[d]));
						}
					}
				} {
					vector<Node *> n_v = p_v->nodes;
					vector<double> w_v = p_v->weights;
					// --
					for (size_t i_v = 0; i_v < p_v->nodes.size(); ++i_v) {
						int    n = n_v[i_v]->nodeIndex;
						double w = w_v[i_v];
						// --
						for (int d = 0; d < D(); ++d) {
							// Cab[D()*n + d] -= w*e_hat[d];
							triplets.push_back(MTriplet(D()*n + d, j, -w*e_hat[d]));
						}
					}
				}
			}
			// Dj += Cab;
		}
	} 

	A.setFromTriplets(triplets.begin(), triplets.end());
	return A; 
}

SparseMatrix SoftIKSolver::calculate_H(const dVector &x, const dVector &alphac) {
	SparseMatrix H_sparse(D()*N(), D()*N());
	// --
	vector<MTriplet> triplets;
	{ 
		dVector alphac_push = mesh->balphac; {
			mesh->balphac = alphac; // (*)
			// mesh->x = x_0; mesh->v = v_0; // NOTE: This won't actually do anything since dynamics contrib to H is constant.
			// --
			auto &E = mesh->energyFunction;
			(SOLVE_DYNAMICS) ? E->setToDynamicsMode(timeStep) : E->setToStaticsMode(0.);
			E->addHessianEntriesTo(triplets, x);
		} mesh->balphac = alphac_push;
	}
	H_sparse.setZero(); // (*)
	H_sparse.setFromTriplets(triplets.begin(), triplets.end());

	int NUM_TRIPLETS = triplets.size(); // (*)
	for (int i = 0; i < NUM_TRIPLETS; ++i) {
		const auto &triplet = triplets[i];
		if (triplet.row() != triplet.col()) { // FORNOW
			triplets.push_back(MTriplet(triplet.col(), triplet.row(), triplet.value()));
		}
	} 

	H_sparse.resize(D()*N(), D()*N());
	H_sparse.setFromTriplets(triplets.begin(), triplets.end());

	return H_sparse; 
}

void SoftIKSolver::construct_alphac_barrierFuncs() {
	this->alphac_barrierFuncs.clear(); 
	for (auto &tendon : mesh->tendons) {
		double ALPHAC_MAX = .33*tendon->get_alphaz();
		alphac_barrierFuncs.push_back(new ZeroCubicQuadratic(1e5, .01, V3D(ALPHAC_MAX, 0.), false, false));
	}
}

////////////////////////////////////////////////////////////////////////////////

int SoftIKSolver::D() { return mesh->D(); } 
int SoftIKSolver::N() { return mesh->N(); } 
int SoftIKSolver::T() { return mesh->tendons.size(); }
 