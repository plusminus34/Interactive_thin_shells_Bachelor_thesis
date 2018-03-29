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
	if (CHECK_IK_GRADIENT) { check_gradient(alphac_curr, x_curr); }
	if (PROJECT) { project(); }
	for (int _ = 0; _ < NUM_ITERS_PER_STEP; ++_) {
		iterate();
	}
	// NOTE!
	// cout << calculate_O(alphac_curr) << endl;
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

			if (Gamma_i < .0005) {
				PROJECTED = true;
				alphac_proj[i] += (.001 - Gamma_i);
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
	dRowVector dOdalphac = calculate_dOdalphac(alphac, x);
	double gamma = calculate_gamma(alphac, dOdalphac);
	return alphac - gamma * dOdalphac.transpose(); 
}

dRowVector SoftIKSolver::calculate_dOdalphac(const dVector &alphac, const dVector &x) {
	dRowVector dQdalphac = calculate_dQdalphac(alphac, x); 
	dRowVector dRdalphac = calculate_dRdalphac(alphac, x); 
	return dQdalphac + dRdalphac;
}

bool SoftIKSolver::check_gradient(const dVector &alphac, const dVector &x) {
		auto Q_wrapper = [this](const dVector &alphac) { return calculate_Q(alphac); };
		auto R_wrapper = [this](const dVector &alphac) { return calculate_R(alphac); };
		cout << "Checking Q..." << endl; bool Q_passes = vector_equality_check(vec_FD(alphac, Q_wrapper, 5e-5), calculate_dQdalphac(alphac, x));
		cout << "Checking R..." << endl; bool R_passes = vector_equality_check(vec_FD(alphac, R_wrapper, 5e-5), calculate_dRdalphac(alphac, x)); 
		return (Q_passes && R_passes);
}

double SoftIKSolver::calculate_gamma(const dVector &alphac, const dRowVector &dOdalphac) {
	double gamma_0 = 1.;
	int maxLineSearchIterations = 10;

	dVector alphac_0 = alphac;
	double O_0 = calculate_O(alphac_0); 
	double gamma_k = gamma_0;
	for (int j = 0; j < maxLineSearchIterations; j++) { 
		dVector alphac_k = alphac_0 - gamma_k * dOdalphac.transpose(); 
		double O_k = calculate_O(alphac_k);

		if (!isfinite(O_k)) {
			error("non-finite O_k detected.");
			O_k = O_0 + 1.0;
		}

		if (O_k > O_0) {
			gamma_k /= 2.0;
		} else {
			// success
			return gamma_k;
		}
	}

	// error("Line search failed.");
	return gamma_k; 
}

// -- //

dVector SoftIKSolver::x_of_alphac(const dVector &alphac) { // FORNOW
	auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(x_0, v_0, alphac) : mesh->solve_statics(x_0, alphac);
	return xv.first;
}

// -- //

double SoftIKSolver::calculate_O(const dVector &alphac) {
	return calculate_Q(alphac) + calculate_R(alphac);
}

double SoftIKSolver::calculate_Q(const dVector &alphac) {
	return (LINEAR_APPROX) ? calculate_Q_approx(alphac) : calculate_Q_formal(alphac);
}
 
double SoftIKSolver::calculate_Q_formal(const dVector &alphac) {
	dVector x = x_of_alphac(alphac);
	double Q_formal  = calculate_Q_of_x(x);
	return Q_formal;
}

double SoftIKSolver::calculate_Q_approx(const dVector &alphac) { 
	dVector dalphac = alphac - alphac_curr;
	dVector x_approx = x_curr + dxdalphac * dalphac; 
	double Q_approx = calculate_Q_of_x(x_approx); 
	return Q_approx;
}

double SoftIKSolver::calculate_R(const dVector &alphac) {

	double ret = 0.;

	{
		for (int i = 0; i < T(); ++i) {
			ret += alphac_barrierFuncs[i]->computeValue(alphac[i]);
		}
	}

	if (HONEY_alphac) {
		for (int i = 0; i < T(); ++i) {
			ret += alphac_honeyFunc->computeValue(alphac[i] - alphac_curr[i]);
		}
	}

	if (REGULARIZE_alphac) {
		for (int i = 0; i < T(); ++i) {
			ret += alphac_regFunc->computeValue(alphac[i]);
		}
	}

	return ret; 
}

dRowVector SoftIKSolver::calculate_dQdalphac(const dVector &alphac, const dVector &x) {
	return calculate_dQdx(alphac, x) * calculate_dxdalphac(alphac, x);
}
 
dRowVector SoftIKSolver::calculate_dQdx(const dVector &alphac, const dVector &x) { 
	dRowVector dQdx; dQdx.setZero(DN());

	if (SPEC_FREESTYLE) {
		dQdx += (x - mesh->x_prime)*Z().asDiagonal();
	}

	if (SPEC_COM) {
		V3D DeltaCOM_ = V3D(COMp, mesh->get_COM(x));
		dRowVector dQdCOM; dQdCOM.setZero(D()); // dRowVector dQdCOM = DeltaCOM;
		for (int i = 0; i < D(); ++i) { dQdCOM[i] = DeltaCOM_[i]; }
		// --
		dQdx += dQdCOM * mesh->get_dCOMdx();
	}

	return dQdx;
}

SparseMatrix SoftIKSolver::calculate_dxdalphac(const dVector &alphac, const dVector &x) {
	SparseMatrix dxdtau; // H^{-1} A
	{
		SparseMatrix A = calculate_A(x);
		SparseMatrix H = calculate_H(x, alphac);

		Eigen::SimplicialLDLT<SparseMatrix> ldlt;
		ldlt.compute(H); // TODO: Can split and then one half you only need to do when sparsity structure changes.
		if (ldlt.info() != Eigen::Success) { error("Eigen::SimplicialLDLT decomposition failed."); } 
		dxdtau = ldlt.solve(A);
		if (ldlt.info() != Eigen::Success) { error("Eigen::SimplicialLDLT solve failed."); }
	}

	dVector dtaudalphac_diag; // dVector dtaudGamma_diag;
	{
		resize_zero(dtaudalphac_diag, T());
		for (int j = 0; j < T(); ++j) {
			auto &tendon = mesh->tendons[j];
			auto *E_j = tendon->tendon_energy_model();
			double Gamma_j = tendon->get_Gamma(x, alphac);
			dtaudalphac_diag[j] = E_j->computeSecondDerivative(Gamma_j);
		}
	}
 
	dxdalphac = dxdtau * dtaudalphac_diag.asDiagonal(); // NOTE: Store for use in LINEAR_APPROX

	return dxdalphac;
}

dRowVector SoftIKSolver::calculate_dRdalphac(const dVector &alphac, const dVector &x) { 
	dRowVector dRdalphac; resize_zero(dRdalphac, T());

	{
		dRowVector dalphacBarrierdalphac; resize_zero(dalphacBarrierdalphac, T());
		for (int i = 0; i < T(); ++i) {
			dalphacBarrierdalphac[i] += alphac_barrierFuncs[i]->computeDerivative(alphac[i]);
		}
		dRdalphac += dalphacBarrierdalphac;
	}

	if (HONEY_alphac) {
		dRowVector dalphacHoneydalphac; resize_zero(dalphacHoneydalphac, T());
		{
			for (int i = 0; i < T(); ++i) {
				dalphacHoneydalphac[i] += alphac_honeyFunc->computeDerivative(alphac[i] - alphac_curr[i]);
			}
		} 
		dRdalphac += dalphacHoneydalphac;
	}

	if (REGULARIZE_alphac) { 
		dRowVector dalphacRegdalphac; resize_zero(dalphacRegdalphac, T());
		{
			for (int i = 0; i < T(); ++i) {
				dalphacRegdalphac[i] += alphac_regFunc->computeDerivative(alphac[i]);
			}
		} 
		dRdalphac += dalphacRegdalphac;
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

double SoftIKSolver::calculate_Q_of_x(const dVector &x) {
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
		double ALPHAC_MAX = .66*tendon->get_alphaz();
		alphac_barrierFuncs.push_back(new ZeroCubicQuadratic(1e5, .01, V3D(ALPHAC_MAX, 0.), false, false));
	}
}

////////////////////////////////////////////////////////////////////////////////

int SoftIKSolver::D() { return mesh->D(); } 
int SoftIKSolver::N() { return mesh->N(); } 
int SoftIKSolver::DN() { return D()*N(); } 
int SoftIKSolver::T() { return mesh->tendons.size(); }
 