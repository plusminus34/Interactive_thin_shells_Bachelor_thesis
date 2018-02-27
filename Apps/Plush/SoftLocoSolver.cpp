#include "SoftLocoSolver.h"
#include <PlushHelpers/helpers_star.h>
#include <Eigen/SparseCholesky>

SoftLocoSolver::SoftLocoSolver(SimulationMesh *mesh) {
	this->mesh = mesh; 
	x_0 = mesh->x;
	v_0 = mesh->v;

	resize_zero(alphac_curr, T());     // FORNOW
	x_curr = x_of_alphac(alphac_curr); // FORNOW


	for (int _ = 0; _ < K; ++_) { dVector ZERO_; resize_zero(ZERO_, T()); alphacJ_curr.push_back(ZERO_); }
	xJ_curr = xJ_of_alphacJ(alphacJ_curr); 

	for (int _ = 0; _ < K; ++_) { SPEC_FREESTYLE_J.push_back(false); }
	for (int _ = 0; _ < K; ++_) { SPEC_COM_J.push_back(true); }
	for (int i = 0; i < K; ++i) { COMpJ.push_back(mesh->get_COM(xJ_curr[i])); }

	construct_alphac_barrierFuncs(); 
	COMp_FORNOW = mesh->get_COM(mesh->X); 

	cout << "Zalphac_curr: " << alphac_curr.transpose() << endl;
	cout << " alphac_curr: " << alphacJ_curr[0].transpose() << endl;
}

void SoftLocoSolver::draw() { 
	/*
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
		glColorMask(1, 1, 1, 1);
	} glDisable(GL_STENCIL_TEST);
	*/

	/*
	// Shade Overlay
	{
		const auto sloppy_shade = [this]() {
			// glColor4d(0., 0., 0., .33 * exp(1)*dfrac(1, K));
			glColor4d(0., 0., 0., .2);
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
	*/

	/*
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
	*/

	// Widget
	glMasterPush(); {
		glDisable(GL_STENCIL_TEST);
		glDisable(GL_DEPTH_TEST);
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

	// Debug
	glMasterPush(); {
		glDisable(GL_STENCIL_TEST);
		glDisable(GL_DEPTH_TEST);
		glLineWidth(6);
		glTranslated(1., 0., 0.);
		set_color(RATIONALITY);
		glBegin(GL_LINE_STRIP); {
			for (auto &bs : mesh->boundary_simplices) {
				for (auto &node : bs->nodes) {
					glP3D(node->getCoordinates(x_curr));
				}
			}
		} glEnd();
	} glMasterPop();

	// Debug
	glMasterPush(); {
		glDisable(GL_STENCIL_TEST);
		glDisable(GL_DEPTH_TEST);
		glTranslated(1., 0., 0.);
		glLineWidth(3); 
		set_color(ORCHID);
		glBegin(GL_LINE_STRIP); {
			for (auto &bs : mesh->boundary_simplices) {
					for (auto &node : bs->nodes) {
						glP3D(node->getCoordinates(xJ_curr[SELECTED_FRAME_i]));
					}
			}
		} glEnd();
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
	if (PROJECT) { project(); projectJ(); }
	for (int _ = 0; _ < NUM_ITERS_PER_STEP; ++_) {
		iterate();
	}
}

void SoftLocoSolver::iterate() {
	// cout << "Zalphac_pre: " << alphac_curr.transpose() << endl;
	// cout << "Zx_pre:      " << x_curr.transpose() << endl;
	// cout << " alphac_pre: " << alphacJ_curr[0].transpose() << endl;
	// cout << " x_pre:      " << xJ_curr[0].transpose() << endl;

	alphac_curr = alphac_next(alphac_curr, x_curr); // FORNOW
	x_curr  = x_of_alphac(alphac_curr); // FORNOW
	// --
	alphacJ_curr = alphacJ_next(alphacJ_curr, xJ_curr);
	xJ_curr = xJ_of_alphacJ(alphacJ_curr);

	// cout << "Zalphac_post: " << alphac_curr.transpose() << endl;
	// cout << "Zx_post:      " << x_curr.transpose() << endl;
	// cout << " alphac_post: " << alphacJ_curr[0].transpose() << endl;
	// cout << " x_post:      " << xJ_curr[0].transpose() << endl;

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

void SoftLocoSolver::projectJ() { 
	dVector alphac_proj = alphacJ_curr[0];
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
	alphacJ_curr[0] = alphac_proj;
	xJ_curr[0]      = x_of_alphac(alphacJ_curr[0]);

	// TODO:!!!
	// alphac_curr = alphac_proj;
	// x_curr  = x_of_alphac(alphac_curr);
	// Traj x_tmp;
	// Traj v_tmp;
	// for (int i = 0; i < K; ++i) {
	// 	dVector *alphac_ptr = &alphacJ_curr[i];
	// 	dVector *x_ptr = &xJ_curr[i];
	// 	const dVector x_start = (x_tmp.empty()) ? x_0 : x_tmp.back();
	// 	const dVector v_start = (v_tmp.empty()) ? v_0 : v_tmp.back();
	// 	// --
	// 	dVector alphac_proj = *alphac_ptr;
	// 	while (true) {
	// 		bool PROJECTED = false;

	// 		auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(x_start, v_start, alphac_proj) : mesh->solve_statics(x_start, alphac_proj);
	// 		dVector x_proj = xv.first;

	// 		for (int i = 0; i < T(); ++i) {
	// 			double Gamma_i;
	// 			Gamma_i = mesh->tendons[i]->get_Gamma(x_proj, alphac_proj);

	// 			if (Gamma_i < 0) {
	// 				PROJECTED = true;
	// 				alphac_proj[i] += abs(Gamma_i) + .001;
	// 			}
	// 		}

	// 		if (!PROJECTED) {
	// 			break;
	// 		}
	// 	}
	// 	auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(x_start, v_start, alphac_proj) : mesh->solve_statics(x_start, alphac_proj);
	// 	x_tmp.push_back(xv.first);
	// 	v_tmp.push_back(xv.second);
	// 	*alphac_ptr = alphac_proj;
	// 	*x_ptr = xv.first;
	// }
}

Traj SoftLocoSolver::alphacJ_next(const Traj &alphacJ, const Traj &xJ) { 
	Traj dOdalphacJ = calculate_dOdalphacJ(alphacJ, xJ);
	double gammaJ = calculate_gammaJ(alphacJ, dOdalphacJ);
	Traj alphacJ_next;
	for (int i = 0; i < K; ++i) {
		alphacJ_next.push_back(alphacJ[i] - gammaJ * dOdalphacJ[i]);
	}
	// cout << " dOdalphac: " << endl << dOdalphacJ[0] << endl;
	// cout << " gamma:     " << endl << gammaJ << endl;
	return alphacJ_next;
}

dVector SoftLocoSolver::alphac_next(const dVector &alphac, const dVector &x) {
	check_alphac_size(alphac);
	check_x_size(x);
	// --
	dVector dOdalphac = calculate_dOdalphac(alphac, x);
	cout << "ZdOdalphac: " << endl << dOdalphac << endl;
	double gamma = calculate_gamma(alphac, dOdalphac);
	cout << "Zgamma:     " << endl << gamma << endl;
	return alphac - gamma * dOdalphac; 
}

double SoftLocoSolver::calculate_gamma(const dVector &alphac, const dVector &dOdalphac) {
	check_alphac_size(alphac);
	// --
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
			return gamma_k;
		}
	}

	// error("Line search failed.");
	return gamma_k; 
}

double SoftLocoSolver::calculate_gammaJ(const Traj &alphacJ, const Traj &dOdalphacJ) {
	double gammaJ_0 = 1.;
	int maxLineSearchIterations = 10;

	Traj alphacJ_0 = alphacJ;
	double O_0 = calculate_OJ(alphacJ_0); 
	double gammaJ_k = gammaJ_0;
	for (int j = 0; j < maxLineSearchIterations; j++) { 
		Traj alphacJ_k; // = alphacJ_0 - gammaJ_k * dOdalphacJ; 
		for (int i = 0; i < K; ++i) {
			alphacJ_k.push_back(alphacJ_0[i] - gammaJ_k * dOdalphacJ[i]);
		}
		double O_k = calculate_OJ(alphacJ_k);

		if (!isfinite(O_k)) {
			error("non-finite O_k detected.");
			O_k = O_0 + 1.0;
		}

		if (O_k > O_0) {
			gammaJ_k /= 2.0;
		} else {
			// success
			return gammaJ_k;
		}
	}

	// error("Line search failed.");
	return gammaJ_k; 
}

// -- //

Traj SoftLocoSolver::xJ_of_alphacJ(const Traj &alphacJ) {
	return solve_trajectory(timeStep, x_0, v_0, alphacJ);
}

dVector SoftLocoSolver::x_of_alphac(const dVector &alphac) { // FORNOW
	// error("DeprecatedError");
	// --
	check_alphac_size(alphac);
	// --
	auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(x_0, v_0, alphac) : mesh->solve_statics(x_0, alphac);
	return xv.first;
}

// -- //

double SoftLocoSolver::calculate_OJ(const Traj &alphacJ) {
	double QJ = calculate_QJ(alphacJ);
	double RJ = calculate_RJ(alphacJ);
	return QJ + RJ;
}

double SoftLocoSolver::calculate_QJ(const Traj &alphacJ) {

	Traj xJ;
	if (!LINEAR_APPROX) {
		xJ = xJ_of_alphacJ(alphacJ);
	} else { 
		Traj dalphacJ;
		for (int i = 0; i < K; ++i) {
			dalphacJ.push_back(alphacJ[i] - alphacJ_curr[i]);
		}

		// FORNOW: (ignoring cross terms) TODO: Tensor Product
		for (int i = 0; i < K; ++i) {
			xJ.push_back(xJ_curr[i] + dxdalphacJ_SAVED[i].transpose() * dalphacJ[i]); 
		} 
	}

	double QJ = 0.;
	for (int i = 0; i < K; ++i) {
		// TODO: Inactive Widgets.
		QJ += calculate_Q_of_x(xJ[i], COMpJ[i]);
	}

	return QJ;
}

double SoftLocoSolver::calculate_RJ(const Traj &alphacJ) {
	double RJ = 0.;
	for (int i = 0; i < K; ++i) {
		RJ += calculate_R(alphacJ[i]);
	}
	return RJ; 
}

double SoftLocoSolver::calculate_O(const dVector &alphac) {
	// error("DeprecatedError");
	//--
	check_alphac_size(alphac);
	// --
	double Q = calculate_Q(alphac);
	double R = calculate_R(alphac);
	return Q + R;
}

double SoftLocoSolver::calculate_Q(const dVector &alphac) {
	// error("DeprecatedError");
	//--
	check_alphac_size(alphac);
	// --
	double Q = (LINEAR_APPROX) ? calculate_Q_approx(alphac) : calculate_Q_formal(alphac);
	return Q;
}
 
double SoftLocoSolver::calculate_Q_formal(const dVector &alphac) {
	// error("DeprecatedError");
	//--
	check_alphac_size(alphac);
	// --
	dVector x = x_of_alphac(alphac);
	double Q_formal  = calculate_Q_of_x(x, COMp_FORNOW);
	return Q_formal;
}

double SoftLocoSolver::calculate_Q_approx(const dVector &alphac) { 
	check_alphac_size(alphac);
	// --
	dVector dalphac = alphac - alphac_curr;
	dVector x_approx = x_curr + dxdalphac_SAVED.transpose() * dalphac; 
	double Q_approx = calculate_Q_of_x(x_approx, COMp_FORNOW); 
	return Q_approx;
}

double SoftLocoSolver::calculate_Q_of_x(const dVector &x, const P3D &COMp) {
	check_x_size(x);
	// --
	// TODO: Freestyle
	// (*) D()-invariant hack
	V3D DeltaCOM_ = V3D(COMp, mesh->get_COM(x));
	return .5*DeltaCOM_.squaredNorm();
}
 
double SoftLocoSolver::calculate_R(const dVector &alphac) {
	check_alphac_size(alphac);

	// --

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

Traj SoftLocoSolver::calculate_dOdalphacJ(const Traj &alphacJ, const Traj &xJ) {
	Traj dQdalphacJ = calculate_dQdalphacJ(alphacJ, xJ);
	Traj dRdalphacJ = calculate_dRdalphacJ(alphacJ);
	Traj dOdalphacJ;
	for (int i = 0; i < K; ++i) {
		dOdalphacJ.push_back(dQdalphacJ[i] + dRdalphacJ[i]);
	}
	return dOdalphacJ;
}

Traj SoftLocoSolver::calculate_dQdalphacJ(const Traj &alphacJ, const Traj &xJ) {

	Traj dQJdalphacJ;
	MatrixNxM dxdalphac_0_SAFE = calculate_dxdalphac(alphacJ[0], xJ[0]);
	dQJdalphacJ.push_back(dxdalphac_0_SAFE * calculate_dQdx(alphacJ[0], xJ[0], COMpJ[0])); 
	dxdalphacJ_SAVED.clear();
	dxdalphacJ_SAVED.push_back(dxdalphac_0_SAFE);
	return dQJdalphacJ;

	// TODO:!!!
	// MatrixNxM calculate_dxdalphac(const dVector &x, const dVector &alphac);
	// MatrixNxM calculate_dxkdxkm1(const dVector &x_k);

	// // FORNOW
	// auto calculate_dxkdxkm1 = [this](const dVector xk, const dVector &alphack) -> MatrixNxM {
	// 	double c = (2. / (timeStep*timeStep));
	// 	MatrixNxM invHk = calculate_H(xk, alphack).toDense().inverse();
	// 	dVector &M_diag = mesh->m;
	// 	MatrixNxM dxkdxkm1 = c * invHk * M_diag.asDiagonal();
	// 	return dxkdxkm1;
	// };
 
	// dVector dQdx_K = calculate_dQdx(alphacJ.back(), xJ.back()); // FORNOW
	// dVector dx1dx0 = calculate_dxkdxkm1(xJ[0], alphacJ[0]);
	// return Traj();

	// Traj dQdalphacJ;
	// for (int k = 0; k < K; ++k) {
	// 	dVector tmp = dQdx_K; // dVector dQdalphac_k:
	// 	for (int j = K; j > k; --j) {
	// 		tmp = calculate_dxkdxkm1(xJ[j], alphacJ[j]) * tmp;
	// 	}
	// 	tmp = calculate_dxdalphac(xJ[k], alphacJ[k]) * tmp;
	// 	dQdalphacJ.push_back(tmp); 
	// }

	// Traj dQJdalphacJ;
	// for (int i = 0; i < K; ++i) {
	// 	dVector dxdalphac_i = calculate_dxdalphac(alphacJ[i], xJ[i]);
	// 	dVector dQJdx_i; resize_zero(dQJdx_i, D()*N());
	// 	for (int j = i; j < K; ++j) {
	// 		dVector dQdx_j = calculate_dQdx(alphacJ[j], xJ[j], COMpJ[j]);
	// 		MatrixNxM dx_jdx_i; dx_jdx_i.setIdentity(D()*N(), D()*N()); // (*)
	// 		for (int k = i + 1; k < j; ++k) {
	// 			dx_jdx_i *= calculate_dxkdxkm1(alphacJ[k], xJ[k]);
	// 		}
	// 		dQJdx_i += dx_jdx_i * dQdx_j;
	// 	}
	// 	dVector dQdalphac_i = dxdalphac_i * dQJdx_i;
	// 	dQJdalphacJ.push_back(dQdalphac_i); 
	// } 
}

Traj SoftLocoSolver::calculate_dRdalphacJ(const Traj &alphacJ) {
	Traj dRdalphacJ;
	for (auto &alphac : alphacJ) {
		dRdalphacJ.push_back(calculate_dRdalphac(alphac));
	}
	return dRdalphacJ;
}

dVector SoftLocoSolver::calculate_dOdalphac(const dVector &alphac, const dVector &x) {
	check_x_size(x);
	check_alphac_size(alphac);
	// --
	dVector dQdalphac = calculate_dQdalphac(alphac, x);
	dVector dRdalphac = calculate_dRdalphac(alphac);
	return dQdalphac + dRdalphac;
} 

dVector SoftLocoSolver::calculate_dQdalphac(const dVector &alphac, const dVector &x) {
	check_x_size(x);
	check_alphac_size(alphac);
	// --
	dxdalphac_SAVED = calculate_dxdalphac(alphac, x);
	return dxdalphac_SAVED * calculate_dQdx(alphac, x, COMp_FORNOW);
}

dVector SoftLocoSolver::calculate_dQdx(const dVector &alphac, const dVector &x, const P3D &COMp) { 
	check_x_size(x);
	check_alphac_size(alphac);
	// --
	dVector dQdx;
	{
		V3D DeltaCOM_ = V3D(COMp, mesh->get_COM(x));
		dVector DeltaCOM; resize_zero(DeltaCOM, D());
		for (int i = 0; i < D(); ++i) { DeltaCOM[i] = DeltaCOM_[i]; }
		// --
		dVector dQdCOM = DeltaCOM;
		dQdx = mesh->get_dCOMdx().transpose() * dQdCOM;
	}

	return dQdx;
}

MatrixNxM SoftLocoSolver::calculate_dxdalphac(const dVector &alphac, const dVector &x) {
	check_x_size(x);
	check_alphac_size(alphac);
	// --
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
 
	MatrixNxM dtaudalphac  = dtaudGamma_diag.asDiagonal(); // NOTE: dGammadalphac = I
	MatrixNxM dxdalphac = dtaudalphac * dxdtau;
	return dxdalphac;
}

dVector SoftLocoSolver::calculate_dRdalphac(const dVector &alphac) { 
	check_alphac_size(alphac);
	// --
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
	check_x_size(x);
	// --
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
	check_x_size(x);
	check_alphac_size(alphac);
	// --
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

bool SoftLocoSolver::check_x_size(const dVector &x) {
	if (x.size() != D() * N()) {
		error("InputDimensionsError");
		return false;
	}
	return true;
}
bool SoftLocoSolver::check_alphac_size(const dVector &alphac) {
	if (alphac.size() != T()) {
		error("InputDimensionsError");
		return false;
	}
	return true;
}

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