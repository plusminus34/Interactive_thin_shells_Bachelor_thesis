#include "SoftLocoSolver.h"
#include <PlushHelpers/helpers_star.h>
#include <Eigen/SparseCholesky>

SoftLocoSolver::SoftLocoSolver(SimulationMesh *mesh) {
	this->mesh = mesh; 
	x_0 = mesh->x;
	v_0 = mesh->v;

	resize_zero(alphac_curr, T());     // FORNOW
	x_curr = x_of_alphac(alphac_curr); // FORNOW

	for (int _ = 0; _ < K; ++_) { dVector SLACK_; resize_fill(SLACK_, T(), -100.); alphacJ_curr.push_back(SLACK_); }
	xJ_curr = xJ_of_alphacJ(alphacJ_curr); 

	for (int _ = 0; _ < K; ++_) { SPEC_FREESTYLE_J.push_back(false); }
	for (int _ = 0; _ < K; ++_) { SPEC_COM_J.push_back(true); }
	for (int i = 0; i < K; ++i) { COMpJ.push_back(mesh->get_COM(xJ_curr[i])); }

	construct_alphac_barrierFuncs(); 
	COMp_FORNOW = mesh->get_COM(mesh->X); 
}

void SoftLocoSolver::draw() {
	// Foreground
	glEnable(GL_STENCIL_TEST); {
		glClear(GL_STENCIL_BUFFER_BIT);
		// --
		glStencilFunc(GL_NOTEQUAL, 1, 1);
		for (int i = -1; i < K; ++i) {
			auto &x = (i == -1) ? x_0 : xJ_curr[i];
			auto &alphac = (i == -1) ? dVector() : alphacJ_curr[i];
			
			glColorMask(1, 1, 1, 1); glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
			mesh->DRAW_TENDONS = (i != -1); {
				mesh->draw(x, alphac);
			} mesh->DRAW_TENDONS = true;

			glColorMask(0, 0, 0, 0); glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
			mesh->draw(x);
		}
	} glDisable(GL_STENCIL_TEST);

	// Shade Overlay
	{
		const auto sloppy_shade = [this]() {
			glColor4d(0., 0., 0., .33 * exp(1)*dfrac(1, K));
			glBegin(GL_QUADS);
			double R = 1000.;
			glP3D(P3D(R, R, 0.));
			glP3D(P3D(-R, R, 0.));
			glP3D(P3D(-R, -R, 0.));
			glP3D(P3D(R, -R, 0.));
			glEnd();
		};
		glEnable(GL_STENCIL_TEST); {
			glDisable(GL_DEPTH_TEST); { // TODO: MasterPush()?
				glClear(GL_STENCIL_BUFFER_BIT);
				// --
				glStencilFunc(GL_NOTEQUAL, 1, 1);
				for (int i = -1; i < K; ++i) {
					auto &x = (i == -1) ? x_0 : xJ_curr[i];
					glColorMask(0, 0, 0, 0); glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
					mesh->draw(x);

					glColorMask(1, 1, 1, 1); glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
					sloppy_shade();
				}
			} glEnable(GL_DEPTH_TEST);
		} glDisable(GL_STENCIL_TEST);
	}

	// Highlight
	glMasterPush(); {
		glDisable(GL_DEPTH_TEST);

		for (auto &bs : mesh->boundary_simplices) {
			set_color(ORCHID);
			glLineWidth(4);
			glBegin(GL_LINE_STRIP); {
				for (auto &node : bs->nodes) {
					glP3D(node->getCoordinates(xJ_curr[SELECTED_FRAME_i]));
				}
			} glEnd();
		}
	} glMasterPop(); 

	// Widget
	glMasterPush(); {
		glDisable(GL_STENCIL_TEST);
		glDisable(GL_DEPTH_TEST);
		// --
		glPointSize(10);
		glLineWidth(5);
		for (int i = 0; i < K; ++i) {
			bool SELECTED = (i == SELECTED_FRAME_i);
			if (SPEC_COM_J[i]) {
				for (auto &GL_PRIMITIVE : { GL_LINES, GL_POINTS }) {
					glBegin(GL_PRIMITIVE); {
						set_color(SELECTED ? PUMPKIN     : DARK_CLAY); glP3D(mesh->get_COM(xJ_curr[i]));
						set_color(SELECTED ? RATIONALITY : CLAY     ); glP3D(COMpJ[i]);
					} glEnd();
				}
			}
		}
	} glMasterPop();
					
}
 
Traj SoftLocoSolver::solve_trajectory(double dt, const dVector &x_0, const dVector &v_0, const Traj &alphacJ) {
	vector<dVector> x_tmp = {};
	vector<dVector> v_tmp = {};

	for (int i = 0; i < K; ++i) {
		const dVector x_im1 = (x_tmp.empty()) ? x_0 : x_tmp.back();
		const dVector v_im1 = (v_tmp.empty()) ? v_0 : v_tmp.back();
		dVector alphac_i = alphacJ[i];
		auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(x_im1, v_im1, alphac_i) : mesh->solve_statics(x_im1, alphac_i);
		x_tmp.push_back(xv.first );
		v_tmp.push_back(xv.second);
	}

	return x_tmp;
}

void SoftLocoSolver::step() {
	 mesh->update_contacts(x_0);
	 if (PROJECT) { project(); }
	 for (int _ = 0; _ < NUM_ITERS_PER_STEP; ++_) {
		 iterate();
	 }
}

void SoftLocoSolver::iterate() {
	alphacJ_curr = alphacJ_next(alphacJ_curr, xJ_curr);
	xJ_curr = xJ_of_alphacJ(alphacJ_curr);
	// alphac_curr = alphac_next(alphac_curr, x_curr);
	// x_curr  = x_of_alphac(alphac_curr); 
}

void SoftLocoSolver::project() { 
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

Traj SoftLocoSolver::alphacJ_next(const Traj &alphacJ, const Traj &xJ) { 
	// TODO: Some approach will work here.
	//
		//Traj dOdalphacJ;
		//for (size_t i = 0; i < alphacJ.size(); ++i) {
			//dOdalphacJ.push_back(calculate_dOdalphac(alphacJ[i], xJ[i]));
		//}
	// double gamma = calculate_gamma(alphac, dOdalphac);
	// return alphac - gamma * dOdalphac; 
	// Traj ret;
	// for (int _ = 0; _ < K; ++_) { ret.push_back(alphac_curr); }
	return alphacJ;
}

dVector SoftLocoSolver::alphac_next(const dVector &alphac, const dVector &x) {
	dVector dOdalphac = calculate_dOdalphac(alphac, x);
	double gamma = calculate_gamma(alphac, dOdalphac);
	return alphac - gamma * dOdalphac; 
}

dVector SoftLocoSolver::calculate_dOdalphac(const dVector &alphac, const dVector &x) {
	// dVector dQdalphac = calculate_dQdalphac(alphac, x, true);
	dVector dQdalphac = calculate_dxdalphac(alphac, x) * calculate_dQdx(alphac, x);
	dVector dRdalphac = calculate_dRdalphac(alphac, x);
	return dQdalphac + dRdalphac;
}

double SoftLocoSolver::calculate_gamma(const dVector &alphac, const dVector &dOdalphac) {
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

Traj SoftLocoSolver::xJ_of_alphacJ(const Traj &alphacJ) {
	return solve_trajectory(timeStep, x_0, v_0, alphacJ);
}

dVector SoftLocoSolver::x_of_alphac(const dVector &alphac) { // FORNOW
	auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(x_0, v_0, alphac) : mesh->solve_statics(x_0, alphac);
	return xv.first;
}

// -- //

double SoftLocoSolver::calculate_O(const dVector &alphac) {
	double Q = calculate_Q(alphac);
	double R = calculate_R(alphac);
	return Q + R;
}

double SoftLocoSolver::calculate_Q(const dVector &alphac) {
	double Q = (LINEAR_APPROX) ? calculate_Q_approx(alphac) : calculate_Q_formal(alphac);
	return Q;
}
 
double SoftLocoSolver::calculate_Q_formal(const dVector &alphac) {
	dVector x = x_of_alphac(alphac);
	double Q_formal  = calculate_Q_of_x(x);
	return Q_formal;
}

double SoftLocoSolver::calculate_Q_approx(const dVector &alphac) { 
	dVector dalphac = alphac - alphac_curr;
	dVector x_approx = x_curr + dxdalphac.transpose() * dalphac; 
	double Q_approx = calculate_Q_of_x(x_approx); 
	return Q_approx;
}

double SoftLocoSolver::calculate_Q_of_x(const dVector &x) {
	// TODO: Freestyle
	// (*) D()-invariant hack
	V3D DeltaCOM_ = V3D(COMp_FORNOW, mesh->get_COM(x));
	return .5*DeltaCOM_.squaredNorm();
}
 

double SoftLocoSolver::calculate_R(const dVector &alphac) {

	double ret = 0.;

	{
		for (int i = 0; i < T(); ++i) {
			ret += alphac_barrierFuncs[i]->computeValue(alphac[i]);
		}
	}

	if (REGULARIZE_alphac) {
		for (int i = 0; i < T(); ++i) {
			ret += alphac_regFunc->computeValue(alphac[i]);
		}
	}

	return ret; 
}

Traj SoftLocoSolver::calculate_dQdalphacJ(const Traj &alphacJ, const Traj &xJ) {

	// MatrixNxM calculate_dxdalphac(const dVector &x, const dVector &alphac);
	// MatrixNxM calculate_dxkdxkm1(const dVector &x_k);

	// FORNOW
	auto calculate_dxkdxkm1 = [this](const dVector xk, const dVector &alphack) {
		double c = (2. / (timeStep*timeStep));
		MatrixNxM invHk = calculate_H(xk, alphack).toDense().inverse();
		dVector &M = mesh->m;
		return c * invHk * M.asDiagonal();
	};
 

	dVector dQdx_K = calculate_dQdx(alphacJ.back(), xJ.back()); // FORNOW

	Traj dQdalphacJ;
	for (size_t k = 0; k < alphacJ.size(); ++k) {
		dVector tmp = dQdx_K; // dVector dQdalphac_k:
		for (size_t j = K; j > k; --j) {
			tmp *= calculate_dxkdxkm1(xJ[j], alphacJ[j]);
		}
		tmp *= calculate_dxdalphac(xJ[k], alphacJ[k]);
		dQdalphacJ.push_back(tmp);
		
	}
	return dQdalphacJ;
}

dVector SoftLocoSolver::calculate_dQdx(const dVector &alphac, const dVector &x) { 
	dVector dQdx;
	{
		V3D DeltaCOM_ = V3D(COMp_FORNOW, mesh->get_COM(x));
		dVector DeltaCOM; resize_zero(DeltaCOM, D());
		for (int i = 0; i < D(); ++i) { DeltaCOM[i] = DeltaCOM_[i]; }
		// --
		dVector dQdCOM = DeltaCOM;
		dQdx = mesh->get_dCOMdx().transpose() * dQdCOM;
	}

	return dQdx;
}

MatrixNxM SoftLocoSolver::calculate_dxdalphac(const dVector &alphac, const dVector &x) {
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

dVector SoftLocoSolver::calculate_dRdalphac(const dVector &alphac, const dVector &x) { 
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
 
	return dRdalphac; 
}
 
SparseMatrix SoftLocoSolver::calculate_A(const dVector &x) {
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

SparseMatrix SoftLocoSolver::calculate_H(const dVector &x, const dVector &alphac) {
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

void SoftLocoSolver::construct_alphac_barrierFuncs() {
	this->alphac_barrierFuncs.clear(); 
	for (auto &tendon : mesh->tendons) {
		double ALPHAC_MAX = .33*tendon->get_alphaz();
		alphac_barrierFuncs.push_back(new ZeroCubicQuadratic(1e5, .01, V3D(ALPHAC_MAX, 0.), false, false));
	}
}

////////////////////////////////////////////////////////////////////////////////

int SoftLocoSolver::D() { return mesh->D(); } 
int SoftLocoSolver::N() { return mesh->N(); } 
int SoftLocoSolver::T() { return mesh->tendons.size(); }
 
/*
	// bool REPLAY = false;
	// int PLAYBACK_i = 0;

	} else {
		mesh->DRAW_TENDONS = false;
		mesh->draw(x_0);

		mesh->DRAW_TENDONS = true;
		int i = ++PLAYBACK_i % xJ_curr.size();
		mesh->draw(xJ_curr[i], alphacJ_curr[i]);
	} 
*/