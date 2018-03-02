#include "SoftLocoSolver.h"
#include <PlushHelpers/helpers_star.h>
#include <Eigen/SparseCholesky>
#include "XYPlot.h"

SoftLocoSolver::SoftLocoSolver(SimulationMesh *mesh) {
	this->mesh = mesh; 
	x_0 = mesh->x;
	v_0 = mesh->v;

	resize_zero(u_curr, T());     // FORNOW
	x_curr = x_of_u(u_curr); // FORNOW


	for (int _ = 0; _ < K; ++_) { dVector ZERO_; resize_zero(ZERO_, T()); uJ_curr.push_back(ZERO_); }
	xJ_curr = xJ_of_uJ(uJ_curr); 

	for (int _ = 0; _ < K; ++_) { SPEC_FREESTYLE_J.push_back(false); }
	for (int _ = 0; _ < K; ++_) { SPEC_COM_J.push_back(true); }
	for (int i = 0; i < K; ++i) { COMpJ.push_back(mesh->get_COM(xJ_curr[i])); }

	construct_u_barrierFuncs(); 
	COMp_FORNOW = mesh->get_COM(mesh->X); 

	// cout << "Zu_curr: " << u_curr.transpose() << endl;
	// cout << " u_curr: " << uJ_curr[0].transpose() << endl;
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
			auto &u = (i == -1) ? dVector() : uJ_curr[i];
			
			glColorMask(1, 1, 1, 1); glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
			mesh->DRAW_TENDONS = (i != -1); {
				mesh->draw(x, u);
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
			// bool SELECTED = (i == SELECTED_FRAME_i);
			bool SELECTED = true;
			// if (i != 0) { continue; } // !!!
			if (i != K - 1) { continue; } // !!!
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
		glPointSize(10);
		glTranslated(1., 0., 0.);
		set_color(HENN1NK);
		glBegin(GL_LINE_STRIP); {
			for (auto &bs : mesh->boundary_simplices) {
				for (auto &node : bs->nodes) {
					glP3D(node->getCoordinates(x_curr));
				}
			}
		} glEnd();
		glBegin(GL_POINTS); {
			glP3D(mesh->get_COM(x_curr)); 
		} glEnd();
	} glMasterPop();

	/* // Debug
	glMasterPush(); {
		glDisable(GL_STENCIL_TEST);
		glDisable(GL_DEPTH_TEST);
		glTranslated(1., 0., 0.);
		glLineWidth(3); 
		set_color(ORCHID);
		glBegin(GL_LINE_STRIP); {
			for (auto &bs : mesh->boundary_simplices) {
					for (auto &node : bs->nodes) {
						glP3D(node->getCoordinates(xJ_curr[0]));
					}
			}
		} glEnd();
	} glMasterPop(); */

	// Debug
	glMasterPush(); {
		glDisable(GL_STENCIL_TEST);
		glDisable(GL_DEPTH_TEST);
		glTranslated(1., 0., 0.);

		for (int i = 0; i < K; ++i) {
			set_color(color_swirl(.5*dfrac(i, K), ORCHID, BLACK));
			// if (i != 0) { set_color(LIGHT_CLAY); } // !!!
			if (i != K - 1) { set_color(LIGHT_CLAY); } // !!!
			glLineWidth(2); 
			glBegin(GL_LINE_STRIP); {
				for (auto &bs : mesh->boundary_simplices) {
					for (auto &node : bs->nodes) {
						glP3D(node->getCoordinates(xJ_curr[i]));
					}
				}
			} glEnd();
			// --
			glPointSize(10);
			glLineWidth(5); 
			if (i != K - 1) { continue;  } // !!!
			for (auto &GL_PRIMITIVE : { GL_POINTS, GL_LINES }) {
				glBegin(GL_PRIMITIVE); {
					// set_color(color_swirl(.5*dfrac(i, K), ORCHID, BLACK));
					set_color(PUMPKIN);
					glP3D(mesh->get_COM(xJ_curr[i]));
					// set_color(color_swirl(.5*dfrac(i, K), RATIONALITY, BLACK));
					set_color(RATIONALITY);
					glP3D(COMpJ[i]);
				} glEnd();
			}
		}
		
	} glMasterPop(); 

	auto time = linspace(K, 0., 1.);
	vector<XYPlot*> plots;
	for (int t = 0; t < T(); ++t) {
		vector<double> F_t;
		for (int k = 0; k < K; ++k) {
			F_t.push_back(uJ_curr[k][t] / mesh->balphaz[t]);
		}
		XYPlot *plot = new XYPlot(time, F_t);
		plot->SPEC_COLOR = kelly_color(t);
		plots.push_back(plot);
	}
	dVector ONE;  resize_fill(ONE,  K, 1);
	dVector ZERO; resize_zero(ZERO, K);
	plots.push_back(new XYPlot(time, dVector2vecDouble(-ONE)));
	plots.push_back(new XYPlot(time, dVector2vecDouble(ZERO)));
	plots.push_back(new XYPlot(time, dVector2vecDouble(ONE)));
	XYPlot::uniformize_axes(plots);
	for (auto &plot : plots) {*plot->origin = P3D(-1.5, -1.);
							  *plot->top_right = P3D(-.5, 1.);
	};
	for (auto &plot : plots) (plot->draw());
	for (auto &plot : plots) {
		delete plot;
	}
					
}
 
Traj SoftLocoSolver::solve_trajectory(double dt, const dVector &x_0, const dVector &v_0, const Traj &uJ) {
	vector<dVector> x_tmp = {};
	vector<dVector> v_tmp = {};

	for (int i = 0; i < K; ++i) {
		const dVector x_im1 = (x_tmp.empty()) ? x_0 : x_tmp.back();
		const dVector v_im1 = (v_tmp.empty()) ? v_0 : v_tmp.back();
		dVector u_i = uJ[i];
		auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(x_im1, v_im1, u_i) : mesh->solve_statics(x_im1, u_i);
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
	// cout << "Zu_pre: " << u_curr.transpose() << endl;
	// cout << "Zx_pre:      " << x_curr.transpose() << endl;
	// cout << " u_pre: " << uJ_curr[0].transpose() << endl;
	// cout << " x_pre:      " << xJ_curr[0].transpose() << endl;

	u_curr = u_next(u_curr, x_curr); // FORNOW
	x_curr  = x_of_u(u_curr); // FORNOW
	// --
	uJ_curr = uJ_next(uJ_curr, xJ_curr);
	xJ_curr = xJ_of_uJ(uJ_curr);

	// cout << "Zu_post: " << u_curr.transpose() << endl;
	// cout << "Zx_post:      " << x_curr.transpose() << endl;
	// cout << " u_post: " << uJ_curr[0].transpose() << endl;
	// cout << " x_post:      " << xJ_curr[0].transpose() << endl;

}

void SoftLocoSolver::project() { 
	dVector u_proj = u_curr;
	while (true) {
		bool PROJECTED = false;

		dVector x_proj = x_of_u(u_proj);

		for (int i = 0; i < T(); ++i) { 
			double Gamma_i;
			Gamma_i = mesh->tendons[i]->get_Gamma(x_proj, u_proj);

			if (Gamma_i < .0005) {
				PROJECTED = true;
				u_proj[i] += (.001 - Gamma_i);
			}
		}

		if (!PROJECTED) {
			break;
		}
	} 
	u_curr = u_proj;
	x_curr  = x_of_u(u_curr);
}

void SoftLocoSolver::projectJ() { 

	/*
	dVector u_proj = uJ_curr[0];
	while (true) {
		bool PROJECTED = false;

		dVector x_proj = x_of_u(u_proj);

		for (int i = 0; i < T(); ++i) { 
			double Gamma_i;
			Gamma_i = mesh->tendons[i]->get_Gamma(x_proj, u_proj);

			if (Gamma_i < 0) {
				PROJECTED = true;
				u_proj[i] += abs(Gamma_i) + .001;
			}
		}

		if (!PROJECTED) {
			break;
		}
	} 
	uJ_curr[0] = u_proj;
	xJ_curr[0]      = x_of_u(uJ_curr[0]);
	*/

	// TODO:!!!
	Traj x_tmp;
	Traj v_tmp;
	for (int i = 0; i < K; ++i) {
		dVector *u_ptr = &uJ_curr[i];
		dVector *x_ptr = &xJ_curr[i];
		const dVector x_start = (x_tmp.empty()) ? x_0 : x_tmp.back();
		const dVector v_start = (v_tmp.empty()) ? v_0 : v_tmp.back();
		// --
		dVector u_proj = *u_ptr;
		while (true) {
			bool PROJECTED = false;

			auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(x_start, v_start, u_proj) : mesh->solve_statics(x_start, u_proj);
			dVector x_proj = xv.first;

			for (int i = 0; i < T(); ++i) {
				double Gamma_i;
				Gamma_i = mesh->tendons[i]->get_Gamma(x_proj, u_proj);

				if (Gamma_i < .0005) {
					PROJECTED = true;
					u_proj[i] += (.001 - Gamma_i);
				}
			}

			if (!PROJECTED) {
				break;
			}
		}
		auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(x_start, v_start, u_proj) : mesh->solve_statics(x_start, u_proj);
		x_tmp.push_back(xv.first);
		v_tmp.push_back(xv.second);
		*u_ptr = u_proj;
		*x_ptr = xv.first;
	}
}

Traj SoftLocoSolver::uJ_next(const Traj &uJ, const Traj &xJ) { 
	Traj dOduJ = calculate_dOduJ(uJ, xJ);
	double gammaJ = calculate_gammaJ(uJ, dOduJ);
	Traj uJ_next;
	for (int i = 0; i < K; ++i) {
		uJ_next.push_back(uJ[i] - gammaJ * dOduJ[i]);
	}
	// cout << " dOdu: " << endl << dOduJ[0] << endl;
	// cout << " gamma:     " << endl << gammaJ << endl;
	return uJ_next;
}

dVector SoftLocoSolver::u_next(const dVector &u, const dVector &x) {
	check_u_size(u);
	check_x_size(x);
	// --
	dVector dOdu = calculate_dOdu(u, x);
	// cout << "ZdOdu: " << endl << dOdu << endl;
	double gamma = calculate_gamma(u, dOdu);
	// cout << "Zgamma:     " << endl << gamma << endl;
	return u - gamma * dOdu; 
}

double SoftLocoSolver::calculate_gamma(const dVector &u, const dVector &dOdu) {
	check_u_size(u);
	// --
	double gamma_0 = 1.;
	int maxLineSearchIterations = 10;

	dVector u_0 = u;
	double O_0 = calculate_O(u_0); 
	double gamma_k = gamma_0;
	for (int j = 0; j < maxLineSearchIterations; j++) { 
		dVector u_k = u_0 - gamma_k * dOdu; 
		double O_k = calculate_O(u_k);

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

double SoftLocoSolver::calculate_gammaJ(const Traj &uJ, const Traj &dOduJ) {
	double gammaJ_0 = 1.;
	int maxLineSearchIterations = 10;

	Traj uJ_0 = uJ;
	double O_0 = calculate_OJ(uJ_0); 
	double gammaJ_k = gammaJ_0;
	for (int j = 0; j < maxLineSearchIterations; j++) { 
		Traj uJ_k; // = uJ_0 - gammaJ_k * dOduJ; 
		for (int i = 0; i < K; ++i) {
			uJ_k.push_back(uJ_0[i] - gammaJ_k * dOduJ[i]);
		}
		double O_k = calculate_OJ(uJ_k);

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

Traj SoftLocoSolver::xJ_of_uJ(const Traj &uJ) {
	return solve_trajectory(timeStep, x_0, v_0, uJ);
}

dVector SoftLocoSolver::x_of_u(const dVector &u) { // FORNOW
	// error("DeprecatedError");
	// --
	check_u_size(u);
	// --
	auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(x_0, v_0, u) : mesh->solve_statics(x_0, u);
	return xv.first;
}

// -- //

double SoftLocoSolver::calculate_OJ(const Traj &uJ) {
	double QJ = calculate_QJ(uJ);
	double RJ = calculate_RJ(uJ);
	return QJ + RJ;
}

double SoftLocoSolver::calculate_QJ(const Traj &uJ) {

	Traj xJ;
	if (!LINEAR_APPROX) {
		xJ = xJ_of_uJ(uJ);
	} else { 
		Traj duJ;
		for (int i = 0; i < K; ++i) {
			duJ.push_back(uJ[i] - uJ_curr[i]);
		}

		// FORNOW: (ignoring cross terms) TODO: Tensor Product
		for (int i = 0; i < K; ++i) {
			xJ.push_back(xJ_curr[i] + dxduJ_SAVED[i].transpose() * duJ[i]); 
		} 
	}

	double QJ = 0.;
	for (int i = 0; i < K; ++i) {
		// if (i != 0) { continue; } // !!!
		if (i != K - 1) { continue; } // !!!
		// TODO: Inactive Widgets.
		QJ += calculate_Q_of_x(xJ[i], COMpJ[i]);
	}

	return QJ;
}

double SoftLocoSolver::calculate_RJ(const Traj &uJ) {
	double RJ = 0.;
	for (int i = 0; i < K; ++i) {
		RJ += calculate_R(uJ[i]);
	}
	return RJ; 
}

double SoftLocoSolver::calculate_O(const dVector &u) {
	// error("DeprecatedError");
	//--
	check_u_size(u);
	// --
	double Q = calculate_Q(u);
	double R = calculate_R(u);
	return Q + R;
}

double SoftLocoSolver::calculate_Q(const dVector &u) {
	// error("DeprecatedError");
	//--
	check_u_size(u);
	// --
	double Q = (LINEAR_APPROX) ? calculate_Q_approx(u) : calculate_Q_formal(u);
	return Q;
}
 
double SoftLocoSolver::calculate_Q_formal(const dVector &u) {
	// error("DeprecatedError");
	//--
	check_u_size(u);
	// --
	dVector x = x_of_u(u);
	double Q_formal  = calculate_Q_of_x(x, COMp_FORNOW);
	return Q_formal;
}

double SoftLocoSolver::calculate_Q_approx(const dVector &u) { 
	check_u_size(u);
	// --
	dVector du = u - u_curr;
	dVector x_approx = x_curr + dxdu_SAVED.transpose() * du; 
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
 
double SoftLocoSolver::calculate_R(const dVector &u) {
	check_u_size(u);

	// --

	double ret = 0.;

	{
		for (int i = 0; i < T(); ++i) {
			ret += u_barrierFuncs[i]->computeValue(u[i]);
		}
	}

	if (REGULARIZE_u) {
		for (int i = 0; i < T(); ++i) {
			ret += u_regFunc->computeValue(u[i]);
		}
	}

	return ret; 
}

Traj SoftLocoSolver::calculate_dOduJ(const Traj &uJ, const Traj &xJ) {
	Traj dQduJ = calculate_dQduJ(uJ, xJ);
	Traj dRduJ = calculate_dRduJ(uJ);
	Traj dOduJ;
	for (int i = 0; i < K; ++i) {
		dOduJ.push_back(dQduJ[i] + dRduJ[i]);
	}
	return dOduJ;
}

Traj SoftLocoSolver::calculate_dQduJ(const Traj &uJ, const Traj &xJ) {

	/*
	Traj dQJduJ;
	MatrixNxM dxdu_0_SAFE = calculate_dxdu(uJ[0], xJ[0]);
	dQJduJ.push_back(dxdu_0_SAFE * calculate_dQdx(uJ[0], xJ[0], COMpJ[0])); 
	dxduJ_SAVED.clear();
	dxduJ_SAVED.push_back(dxdu_0_SAFE);
	return dQJduJ;
	*/

	// TODO:!!!
	// MatrixNxM calculate_dxdu(const dVector &x, const dVector &u);
	// MatrixNxM calculate_dxkdxkm1(const dVector &x_k);

	// dVector dQdx_K = calculate_dQdx(uJ.back(), xJ.back()); // FORNOW
	// dVector dx1dx0 = calculate_dxkdxkm1(xJ[0], uJ[0]);
	// return Traj();

	// Traj dQduJ;
	// for (int k = 0; k < K; ++k) {
	// 	dVector tmp = dQdx_K; // dVector dQdu_k:
	// 	for (int j = K; j > k; --j) {
	// 		tmp = calculate_dxkdxkm1(xJ[j], uJ[j]) * tmp;
	// 	}
	// 	tmp = calculate_dxdu(xJ[k], uJ[k]) * tmp;
	// 	dQduJ.push_back(tmp); 
	// }

	auto unstack_Traj = [this](const dVector &fooJ_d) -> vector<dVector> {
		Traj fooJ;
		int LEN = fooJ_d.size() / K;
		for (int i = 0; i < K; ++i) {
			fooJ.push_back(fooJ_d.segment(i*LEN, LEN));
		} 
		return fooJ;
	};

	auto calculate_dQJduJ_FD = [&](const Traj &uJ, const Traj &xJ) -> Traj {
		auto QJ_wrapper = [&](const dVector uJ_d) -> double {
			return calculate_QJ(unstack_Traj(uJ_d)); 
		};
		return unstack_Traj(vec_FD(stack_vec_dVector(uJ), QJ_wrapper, 5e-4));
	};

	auto calculate_dQJdxJ_FD = [&](const Traj &uJ, const Traj &xJ) -> Traj {

		Traj dQJdxJ_FD;
		for (int i = 0; i < K; ++i) {
			dVector dQJdx_i; resize_zero(dQJdx_i, D()*N());

			if (i == K - 1) { // !!!
				P3D COMp = COMpJ[i];
				auto Q_wrapper = [&](const dVector x) -> double {
					return calculate_Q_of_x(x, COMp);
				};
				dQJdx_i = vec_FD(xJ[i], Q_wrapper, 5e-5);
			}

			dQJdxJ_FD.push_back(dQJdx_i); 
		}
		return dQJdxJ_FD;
	};

	auto Traj_equality_check = [&](const Traj &T1, const Traj &T2) -> void {
		if (T1.size() != T2.size()) { error("SizeMismatchError"); }
		for (size_t i = 0; i < T1.size(); ++i) {
			cout << "--> i = " << i << endl;
			vector_equality_check(T1[i], T2[i]);
		}
	};

	vector<MatrixNxM> dxidxj;
	// TODO

	vector<MatrixNxM> dxidui;
	for (int i = 0; i < K; ++i) {
		dxidui.push_back(calculate_dxdu(uJ[i], xJ[i]));
	}
	
	vector<MatrixNxM> dxiduj;



	dxduJ_SAVED = dxidui; // (***)

	// auto calculate_dxkdxkm1 = [this](const dVector uk, const dVector &xk) -> MatrixNxM {
	// 	double c = (2. / (timeStep*timeStep));
	// 	MatrixNxM invHk = calculate_H(xk, uk).toDense().inverse();
	// 	dVector &M_diag = mesh->m;
	// 	MatrixNxM dxkdxkm1 = c * invHk * M_diag.asDiagonal();
	// 	return dxkdxkm1;
	// }; 

	// Traj dQJdxJ;
	// for (int i = 0; i < K; ++i) { 
	// 	dVector dQJdx_i; resize_zero(dQJdx_i, D()*N());
	// 	for (int j = i; j < K; ++j) {
	// 		// if (j != 0) { continue; } // !!!
	// 		if (j != K - 1) { continue; } // !!! => j <- (K - 1)
	// 		// 
	// 		dVector dQdx_j = calculate_dQdx(uJ[j], xJ[j], COMpJ[j]);
	// 		MatrixNxM dx_jdx_i; dx_jdx_i.setIdentity(D()*N(), D()*N()); // (*)
	// 		for (int k = i + 1; k < j; ++k) {
	// 			dx_jdx_i *= calculate_dxkdxkm1(uJ[k], xJ[k]);
	// 		}
	// 		dQJdx_i += dx_jdx_i * dQdx_j;
	// 	}
	// 	dQJdxJ.push_back(dQJdx_i); 
	// } 

	Traj dQJdxJ_FD = calculate_dQJdxJ_FD(uJ, xJ); 

	auto vMvD2Traj = [](const vector<MatrixNxM> &vM, const Traj &vD) {
		Traj ret;
		if (vM.size() != vD.size()) { error("SizeMismatchError"); }
		for (size_t i = 0; i < vM.size(); ++i) {
			ret.push_back(vM[i] * vD[i]);
		}
		return ret; 
	}; 

	Traj STEP0 = calculate_dQJduJ_FD(uJ, xJ); 
	Traj STEP1 = vMvD2Traj(dxidui, dQJdxJ_FD);
	// Traj STEPX = vMvD2Traj(dxJduJ, dQJdxJ);

	Traj_equality_check(STEP0, STEP1);

	return STEP0;
}

Traj SoftLocoSolver::calculate_dRduJ(const Traj &uJ) {
	Traj dRduJ;
	for (auto &u : uJ) {
		dRduJ.push_back(calculate_dRdu(u));
	}
	return dRduJ;
}

dVector SoftLocoSolver::calculate_dOdu(const dVector &u, const dVector &x) {
	check_x_size(x);
	check_u_size(u);
	// --
	dVector dQdu = calculate_dQdu(u, x);
	dVector dRdu = calculate_dRdu(u);
	return dQdu + dRdu;
} 

dVector SoftLocoSolver::calculate_dQdu(const dVector &u, const dVector &x) {
	check_x_size(x);
	check_u_size(u);
	// --
	dxdu_SAVED = calculate_dxdu(u, x);
	return dxdu_SAVED * calculate_dQdx(u, x, COMp_FORNOW);
}

dVector SoftLocoSolver::calculate_dQdx(const dVector &u, const dVector &x, const P3D &COMp) { 
	check_x_size(x);
	check_u_size(u);
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

MatrixNxM SoftLocoSolver::calculate_dxdu(const dVector &u, const dVector &x) {
	check_x_size(x);
	check_u_size(u);
	// --
	MatrixNxM dxdtau; // H^{-1} A
	{
		// X H = A_T
		// H_T X_T = A
		SparseMatrix A_s = calculate_A(x);
		SparseMatrix H_T_s = calculate_H(x, u).transpose();
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
			double Gamma_j = tendon->get_Gamma(x, u);
			dtaudGamma_diag[j] = E_j->computeSecondDerivative(Gamma_j);
		}
	}
 
	MatrixNxM dtaudu  = dtaudGamma_diag.asDiagonal(); // NOTE: dGammadu = I
	MatrixNxM dxdu = dtaudu * dxdtau;
	return dxdu;
}

dVector SoftLocoSolver::calculate_dRdu(const dVector &u) { 
	check_u_size(u);
	// --
	dVector dRdu; resize_zero(dRdu, T());

	{
		dVector duBarrierdu; resize_zero(duBarrierdu, T());
		for (int i = 0; i < T(); ++i) {
			duBarrierdu[i] += u_barrierFuncs[i]->computeDerivative(u[i]);
		}
		dRdu += duBarrierdu;
	}

	if (REGULARIZE_u) { 
		dVector duRegdu; resize_zero(duRegdu, T());
		{
			for (int i = 0; i < T(); ++i) {
				duRegdu[i] += u_regFunc->computeDerivative(u[i]);
			}
		} 
		dRdu += duRegdu;
	}
 
	return dRdu; 
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

SparseMatrix SoftLocoSolver::calculate_H(const dVector &x, const dVector &u) {
	check_x_size(x);
	check_u_size(u);
	// --
	SparseMatrix H_sparse(D()*N(), D()*N());
	// --
	vector<MTriplet> triplets;
	{ 
		dVector u_push = mesh->balphac; {
			mesh->balphac = u; // (*)
			// mesh->x = x_0; mesh->v = v_0; // NOTE: This won't actually do anything since dynamics contrib to H is constant.
			// --
			auto &E = mesh->energyFunction;
			(SOLVE_DYNAMICS) ? E->setToDynamicsMode(timeStep) : E->setToStaticsMode(0.);
			E->addHessianEntriesTo(triplets, x);
		} mesh->balphac = u_push;
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

void SoftLocoSolver::construct_u_barrierFuncs() {
	this->u_barrierFuncs.clear(); 
	for (auto &tendon : mesh->tendons) {
		double ALPHAC_MAX = .66*tendon->get_alphaz();
		u_barrierFuncs.push_back(new ZeroCubicQuadratic(1e5, .01, V3D(ALPHAC_MAX, 0.), false, false));
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
bool SoftLocoSolver::check_u_size(const dVector &u) {
	if (u.size() != T()) {
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
		mesh->draw(xJ_curr[i], uJ_curr[i]);
	} 
*/