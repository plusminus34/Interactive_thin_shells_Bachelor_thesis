#include "SoftLocoSolver.h"
#include <PlushHelpers/helpers_star.h>
#include <Eigen/SparseCholesky>
#include "XYPlot.h"

SoftLocoSolver::SoftLocoSolver(SimulationMesh *mesh) {
	this->mesh = mesh; 
	xm1_curr = mesh->x;
	vm1_curr = mesh->v;
	resize_zero(um1_curr, T()); // FORNOW

	// resize_zero(u_curr, T());     // FORNOW
	// x_curr = x_of_u(u_curr); // FORNOW


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

	// // Debug
	// glMasterPush(); {
	// 	glDisable(GL_STENCIL_TEST);
	// 	glDisable(GL_DEPTH_TEST);
	// 	glLineWidth(6);
	// 	glPointSize(10);
	// 	glTranslated(1., 0., 0.);
	// 	set_color(HENN1NK);
	// 	glBegin(GL_LINE_STRIP); {
	// 		for (auto &bs : mesh->boundary_simplices) {
	// 			for (auto &node : bs->nodes) {
	// 				glP3D(node->getCoordinates(x_curr));
	// 			}
	// 		}
	// 	} glEnd();
	// 	glBegin(GL_POINTS); {
	// 		glP3D(mesh->get_COM(x_curr)); 
	// 	} glEnd();
	// } glMasterPop();

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

		for (int i = K - 1; i >= 0; --i) {
			set_color(color_swirl(dfrac(i, K), ORCHID, RATIONALITY));
			// if (i != 0) { set_color(LIGHT_CLAY); } // !!!
			// if (i != K - 1) { set_color(LIGHT_CLAY); } // !!!
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

	for (int i = 0; i < (int)uJ.size(); ++i) {
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
	// if (!mesh->contacts.empty()) { error("[SoftLocoSolver::step()] NotImplementedError."); }
	// mesh->update_contacts(xm1_curr);
	if (PROJECT) {
		// project();
		projectJ();
	}
	for (int _ = 0; _ < NUM_ITERS_PER_STEP; ++_) {
		iterate();
	}
}

void SoftLocoSolver::iterate() {
	// cout << "Zu_pre: " << u_curr.transpose() << endl;
	// cout << "Zx_pre:      " << x_curr.transpose() << endl;
	// cout << " u_pre: " << uJ_curr[0].transpose() << endl;
	// cout << " x_pre:      " << xJ_curr[0].transpose() << endl;

	// u_curr = u_next(u_curr, x_curr); // FORNOW
	// x_curr  = x_of_u(u_curr); // FORNOW
	// --
	uJ_curr = uJ_next(uJ_curr, xJ_curr);
	xJ_curr = xJ_of_uJ(uJ_curr);

	// cout << "Zu_post: " << u_curr.transpose() << endl;
	// cout << "Zx_post:      " << x_curr.transpose() << endl;
	// cout << " u_post: " << uJ_curr[0].transpose() << endl;
	// cout << " x_post:      " << xJ_curr[0].transpose() << endl;

}

// void SoftLocoSolver::project() { 
// 	dVector u_proj = u_curr;
// 	while (true) {
// 		bool PROJECTED = false;

// 		dVector x_proj = x_of_u(u_proj);

// 		for (int i = 0; i < T(); ++i) { 
// 			double Gamma_i;
// 			Gamma_i = mesh->tendons[i]->get_Gamma(x_proj, u_proj);

// 			if (Gamma_i < .0005) {
// 				PROJECTED = true;
// 				u_proj[i] += (.001 - Gamma_i);
// 			}
// 		}

// 		if (!PROJECTED) {
// 			break;
// 		}
// 	} 
// 	u_curr = u_proj;
// 	x_curr  = x_of_u(u_curr);
// }

void SoftLocoSolver::projectJ() { 
	// TODO (CHECKME)
	Traj x_tmp;
	Traj v_tmp;
	for (int i = 0; i < K; ++i) {
		const dVector x_start = (x_tmp.empty()) ? xm1_curr : x_tmp.back();
		const dVector v_start = (v_tmp.empty()) ? vm1_curr : v_tmp.back();
		// --
		dVector u_proj = uJ_curr[i];
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
		uJ_curr[i] = u_proj;
		xJ_curr[i] = xv.first;
	}

	if (false) { // VALIDATE_PROJECTION
		Traj xJ_CHECK = solve_trajectory(mesh->timeStep, xm1_curr, vm1_curr, uJ_curr);
		cout << "^^^" << endl;
		Traj_equality_check(xJ_curr, xJ_CHECK);
		cout << "vvv" << endl;
	}
}

Traj SoftLocoSolver::uJ_next(const Traj &uJ, const Traj &xJ) { 
	Traj dOduJ = calculate_dOduJ(uJ, xJ);
	double gammaJ = calculate_gammaJ(uJ, dOduJ);
	Traj uJ_next;
	for (int i = 0; i < K; ++i) {
		uJ_next.push_back(uJ[i] - gammaJ * dOduJ[i]);
	}
	return uJ_next;
}

// dVector SoftLocoSolver::u_next(const dVector &u, const dVector &x) {
// 	check_u_size(u);
// 	check_x_size(x);
// 	// --
// 	dVector dOdu = calculate_dOdu(u, x);
// 	// cout << "ZdOdu: " << endl << dOdu << endl;
// 	double gamma = calculate_gamma(u, dOdu);
// 	// cout << "Zgamma:     " << endl << gamma << endl;
// 	return u - gamma * dOdu; 
// }

// double SoftLocoSolver::calculate_gamma(const dVector &u, const dVector &dOdu) {
// 	check_u_size(u);
// 	// --
// 	double gamma_0 = 1.;
// 	int maxLineSearchIterations = 10;

// 	dVector u_0 = u;
// 	double O_0 = calculate_O(u_0); 
// 	double gamma_k = gamma_0;
// 	for (int j = 0; j < maxLineSearchIterations; j++) { 
// 		dVector u_k = u_0 - gamma_k * dOdu; 
// 		double O_k = calculate_O(u_k);

// 		if (!isfinite(O_k)) {
// 			error("non-finite O_k detected.");
// 			O_k = O_0 + 1.0;
// 		}

// 		if (O_k > O_0) {
// 			gamma_k /= 2.0;
// 		} else {
// 			// success
// 			return gamma_k;
// 		}
// 	}

// 	// error("Line search failed.");
// 	return gamma_k; 
// }

double SoftLocoSolver::calculate_gammaJ(const Traj &uJ, const Traj &dOduJ) {
	double gammaJ_0 = 1.;
	int maxLineSearchIterations = 48;

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

	error("SoftLoco line search failed.");
	return gammaJ_k; 
}

// -- //

Traj SoftLocoSolver::xJ_of_uJ(const Traj &uJ) {
	return solve_trajectory(mesh->timeStep, xm1_curr, vm1_curr, uJ);
}

// dVector SoftLocoSolver::x_of_u(const dVector &u) { // FORNOW
// 	// error("DeprecatedError");
// 	// --
// 	check_u_size(u);
// 	// --
// 	auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(xm1_curr, vm1_curr, u) : mesh->solve_statics(xm1_curr, u);
// 	return xv.first;
// }

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

		// CHECKME
		for (int i = 0; i < K; ++i) {
			MatrixNxM entry = xJ_curr[i];
			for (int j = 0; j < K; ++j) {
				entry += dxiduj_SAVED[i][j].transpose() * duJ[j];
			}
			xJ.push_back(entry);
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

	if (SUBSEQUENT_u) {
		for (int k = 0; k < K; ++k) {
			int km1 = k - 1;
			dVector u_prev = (km1 == -1) ? um1_curr : uJ[km1];
			for (int t = 0; t < T(); ++t) {
				RJ += u_subFunc->computeValue(uJ[k][t] - u_prev[t]);
			}
		}
	}

	return RJ; 
}

// double SoftLocoSolver::calculate_O(const dVector &u) {
// 	// error("DeprecatedError");
// 	//--
// 	check_u_size(u);
// 	// --
// 	double Q = calculate_Q(u);
// 	double R = calculate_R(u);
// 	return Q + R;
// }

// double SoftLocoSolver::calculate_Q(const dVector &u) {
// 	// error("DeprecatedError");
// 	//--
// 	check_u_size(u);
// 	// --
// 	double Q = (LINEAR_APPROX) ? calculate_Q_approx(u) : calculate_Q_formal(u);
// 	return Q;
// }
 
// double SoftLocoSolver::calculate_Q_formal(const dVector &u) {
// 	// error("DeprecatedError");
// 	//--
// 	check_u_size(u);
// 	// --
// 	dVector x = x_of_u(u);
// 	double Q_formal  = calculate_Q_of_x(x, COMp_FORNOW);
// 	return Q_formal;
// }

// double SoftLocoSolver::calculate_Q_approx(const dVector &u) { 
// 	check_u_size(u);
// 	// --
// 	dVector du = u - u_curr;
// 	dVector x_approx = x_curr + dxdu_SAVED.transpose() * du; 
// 	double Q_approx = calculate_Q_of_x(x_approx, COMp_FORNOW); 
// 	return Q_approx;
// }

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

	auto calculate_dQJduJ_FD = [&](const Traj &uJ, const Traj &xJ) -> Traj {
		auto QJ_wrapper = [&](const dVector uJ_d) -> double {
			return calculate_QJ(unstack_Traj(uJ_d)); 
		};
		return unstack_Traj(vec_FD(stack_vec_dVector(uJ), QJ_wrapper, 1e-8));
	};
 
	vector<dVector> dQkdxk_FD; 
	vector<MatrixNxM> dxkdxkm1_FD; 
	vector<vector<MatrixNxM>> dxidxj_FD; 
	vector<MatrixNxM> dxkduk_FD; 
	if (TEST_Q_FD) {
		if (true) {
			for (int k = 0; k < K; ++k) {
				dVector entry; resize_zero(entry, DN());
				if (k == K - 1) {
					auto Q_wrapper = [&](const dVector x) -> double {
						return calculate_Q_of_x(x, COMpJ[k]);
					};
					entry = vec_FD(xJ[k], Q_wrapper, 5e-5);
				}
				dQkdxk_FD.push_back(entry);
			}
		}

		if (true) {
			for (int k = 1; k < K; ++k) {
				dVector xkm2 = (k == 1) ? xm1_curr : xJ[k - 2];
				dVector uk   = uJ[k];
				auto xk_wrapper = [&](const dVector &xkm1) -> dVector {
					dVector vkm1 = (xkm1 - xkm2) / mesh->timeStep; // v_new = (x_new - x_0) / mesh->timeStep;
					auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(xkm1, vkm1, uk) : mesh->solve_statics(xkm1, uk);
					return xv.first;
				};
				dxkdxkm1_FD.push_back(mat_FD(xJ[k - 1], xk_wrapper, 5e-5));
			}
		}

		if (true) {
			for (int i = 0; i < K; ++i) {
				vector<MatrixNxM> row;
				for (int j = 0; j < K; ++j) {
					MatrixNxM entry;
					if (i < j) {
						entry.setZero(DN(), DN());
					} else {
						auto xi_wrapper = [&](const dVector &xj) -> dVector {
							dVector xk   = xj;
							dVector xkm1 = (j == 0) ? xm1_curr : xJ[j - 1];
							dVector vk = (xk - xkm1) / mesh->timeStep; // v_new = (x_new - x_0) / mesh->timeStep;
							for (int k = j; k < i; ++k) {
								dVector ukp1 = uJ[k + 1];
								auto xvp1 = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(xk, vk, ukp1) : mesh->solve_statics(xk, ukp1);
								// --
								xk = xvp1.first;
								vk = xvp1.second;
							}
							return xk;
						};
						entry = mat_FD(xJ[j], xi_wrapper, 5e-5);
					}
					row.push_back(entry);
				}
				dxidxj_FD.push_back(row);
			}
		}

		if (true) {
			for (int k = 0; k < K; ++k) {
				dVector xkm1 = (k == 0) ? xm1_curr : xJ[k - 1];
				dVector vkm1;
				if (k == 0) {
					vkm1 = vm1_curr;
				} else if (k == 1) {
					vkm1 = (xJ[0] - xm1_curr) / mesh->timeStep; 
				} else {
					vkm1 = (xJ[k - 1] - xJ[k - 2]) / mesh->timeStep; 
				}
				auto xk_wrapper = [&](const dVector &uk) -> dVector {
					auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(xkm1, vkm1, uk) : mesh->solve_statics(xkm1, uk);
					return xv.first;
				};
				dxkduk_FD.push_back(mat_FD(uJ[k], xk_wrapper, 5e-5));
			}
		}
	}


	double h = mesh->timeStep;
	MatrixNxM M = mesh->m.asDiagonal();
	MatrixNxM I; I.setIdentity(DN(), DN());
	MatrixNxM f1h2M = (1./(h*h))*M;
	vector<MatrixNxM> vecHinv;
	for (int k = 0; k < K; ++k) {
		dVector x_ctc = (k == 0) ? xm1_curr : xJ[k - 1];
		vecHinv.push_back(calculate_H(xJ[k], uJ[k], x_ctc).toDense().inverse());
	}

	// cout << "dxkdxkm1" << endl;
	vector<MatrixNxM> dGdxkm1_INDEX_at_km1;
	vector<MatrixNxM> dScriptGdxkm1_INDEX_at_km1;
	vector<MatrixNxM> dxkdxkm1_INDEX_AT_km1; {
		for (int k = 1; k < K; ++k) {
			MatrixNxM &Hinv = vecHinv[k];

			MatrixNxM dGkdxkm1; 
			dVector xk   = xJ[k];
			dVector xkm1 = xJ[k - 1];
			mesh->update_contacts(xkm1); // FORNOW

			mat_resize_zero(dGkdxkm1, DN(), DN());
			for (auto &c: mesh->contacts) {
				int i = c->node->nodeIndex;
				P3D xy_k_i   = c->getPosition(xk);
				P3D xy_km1_i = c->getPosition(xkm1);
				double &xk_i   = xy_k_i[0];
				double &xkm1_i = xy_km1_i[0];
				Matrix2x2 block; block.setZero();
				block(0, 0) = -c->b()*c->QT->computeSecondDerivative(xk_i - xkm1_i);
				block(1, 0) = c->b_prime()*c->QT->computeDerivative(xk_i - xkm1_i);
				dGkdxkm1.block<2, 2>(2 * i, 2 * i) += block;
			}
			dGdxkm1_INDEX_at_km1.push_back(dGkdxkm1);

			if (TEST_Q_FD) {
				MatrixNxM dGkdxkm1_FD;
				auto G_wrapper = [&](const dVector xkm1) -> dVector {
					dVector G;
					mesh->update_contacts(xkm1);
					mesh->energyFunction->setToStaticsMode(0.);
					mesh->energyFunction->addGradientTo(G, xk);
					(SOLVE_DYNAMICS) ? mesh->energyFunction->setToDynamicsMode(mesh->timeStep) : mesh->energyFunction->setToStaticsMode(0.);
					return G;
				};
				dGkdxkm1_FD = mat_FD(xkm1, G_wrapper, 5e-5);

				cout << "--> dGkdxkm1" << endl;
				matrix_equality_check(dGkdxkm1_FD, dGkdxkm1);
			}

			MatrixNxM dScriptGdxkm1 = dGkdxkm1 - 2.*f1h2M;
			dScriptGdxkm1_INDEX_at_km1.push_back(dScriptGdxkm1);

			dxkdxkm1_INDEX_AT_km1.push_back(-dScriptGdxkm1*Hinv);
		}
		mesh->update_contacts(mesh->x);
	} 

	// cout << "dxkdxkm2" << endl;
	vector<MatrixNxM> dxkdxkm2_INDEX_AT_km2; {
		for (int k = 2; k < K; ++k) {
			double h = mesh->timeStep;
			MatrixNxM &Hinv = vecHinv[k];
			MatrixNxM &TERM_1 = f1h2M;
			MatrixNxM TERM_2 = dxkdxkm1_INDEX_AT_km1[k - 2] * dScriptGdxkm1_INDEX_at_km1[k - 1];
			dxkdxkm2_INDEX_AT_km2.push_back(-(TERM_1 + TERM_2)*Hinv);
		}
	} 
 
	// cout << "dxidxj" << endl;
	
	// -- // Seed.
	auto &D1 = [&](int k) { return dxkdxkm1_INDEX_AT_km1[k - 1]; };
	auto &D2 = [&](int k) { return dxkdxkm2_INDEX_AT_km2[k - 2]; };
	// --
	vector<vector<MatrixNxM>> dxidxj;
	for (int i = 0; i < K; ++i) {
		vector<MatrixNxM> row;
		for (int j = 0; j < K; ++j) {
			int imj = i - j;
			MatrixNxM entry; entry.setZero(DN(), DN());
			if (imj == 0) {
				entry.setIdentity(DN(), DN());
			} else if (imj == 1) {
				entry = D1(i); // dxkdxkm1_INDEX_AT_km1[i - 1]; // (*)
			} else if (imj == 2) {
				entry = D2(i); // M = dxkdxkm2_INDEX_AT_km2[i - 2]; // (*)
			}
			row.push_back(entry);
		}
		dxidxj.push_back(row);
	}

	// -- // Grow.
	auto &XX = dxidxj;
	auto &scrGxm1 = [&](int k) { return dScriptGdxkm1_INDEX_at_km1[k - 1]; };
	// --
	for (int j = 0; j < K; ++j) {
		for (int i = j + 3; i < K; ++i) { 
			MatrixNxM &Hinv = vecHinv[i];
			// --
			dxidxj[i][j] = -(XX[i - 1][j]*scrGxm1(i) + XX[i - 2][j]*f1h2M)*Hinv; 
		}
	}

	// cout << "dxkduk" << endl;
	vector<MatrixNxM> dxkduk;
	for (int k = 0; k < K; ++k) {
		dVector x_ctc = (k == 0) ? xm1_curr : xJ[k - 1];
		dxkduk.push_back(calculate_dxdu(uJ[k], xJ[k], x_ctc));
	}
	
	// cout << "dQkdxk" << endl;
	vector<dVector> dQkdxk;
	for (int k = 0; k < K; ++k) {
		dVector entry;
		if (k != K - 1) {
			resize_zero(entry, DN());
		} else {
			entry = calculate_dQdx(uJ[k], xJ[k], COMpJ[k]);
		}
		dQkdxk.push_back(entry);
	}

	// cout << "dQiduj" << endl;
	vector<vector<dVector>> dQiduj;
	// dxjduj dxidxj dQidxi
	for (int i = 0; i < K; ++i) {
		vector<dVector> row;
		for (int j = 0; j < K; ++j) {
			row.push_back(dxkduk[j] * dxidxj[i][j] * dQkdxk[i]); // FORNOW
		}
		dQiduj.push_back(row);
	}

	// cout << "dQJduj" << endl;
	Traj STEPX;
	// Traj[j] = sum_i {dQiduj}
	for (int j = 0; j < K; ++j) {
		dVector entry; resize_zero(entry, T());
		for (int i = 0; i < K; ++i) {
			entry += dQiduj[i][j];
		}
		STEPX.push_back(entry);
	}

	if (TEST_Q_FD) {
		cout << "BEG.................................................." << endl;
		cout << "--> dxkduk" << endl;
		MTraj_equality_check(dxkduk_FD, dxkduk);
		// cout << "--> dxkdxkm1" << endl;
		// MTraj_equality_check(dxkdxkm1_FD, dxkdxkm1_INDEX_AT_km1);
		cout << "--> dxidxj" << endl;
		for (int i = 0; i < K; ++i) {
			cout << "----> ii = " << i << endl;
			MTraj_equality_check(dxidxj_FD[i], dxidxj[i]);
		}
		cout << "--> dQkdxk" << endl;
		Traj_equality_check(dQkdxk_FD, dQkdxk);
		cout << "..................................................END" << endl;
	}
 
	if (TEST_Q_FD) {
		cout << "XXX.................................................." << endl;
		Traj STEP0 = calculate_dQJduJ_FD(uJ, xJ);
		Traj_equality_check(STEP0, STEPX);
		cout << "..................................................XXX" << endl;
	}

	dxiduj_SAVED.clear();
	for (int i = 0; i < K; ++i) {
		vector<MatrixNxM> row;
		for (int j = 0; j < K; ++j) {
			row.push_back(dxkduk[j] * dxidxj[i][j]);
		}
		dxiduj_SAVED.push_back(row);;
	}

	return STEPX;
}

Traj SoftLocoSolver::calculate_dRduJ(const Traj &uJ) {
	Traj dRduJ;

	for (auto &u : uJ) {
		dRduJ.push_back(calculate_dRdu(u));
	}
 
	if (SUBSEQUENT_u) {
		for (int k = 0; k < K; ++k) {
			int km1 = k - 1;
			dVector u_prev = (km1 == -1) ? um1_curr : uJ[km1];
			for (int t = 0; t < T(); ++t) {
				dRduJ[k][t]   += u_subFunc->computeDerivative(uJ[k][t] - u_prev[t]);
				if (km1 != -1) { dRduJ[km1][t] -= u_subFunc->computeDerivative(uJ[k][t] - u_prev[t]); }
			}
		}
	}

	if (TEST_R_FD) {
		auto calculate_dRJduJ_FD = [&](const Traj &uJ) -> Traj {
			auto RJ_wrapper = [&](const dVector uJ_d) -> double {
				return calculate_RJ(unstack_Traj(uJ_d)); 
			};
			return unstack_Traj(vec_FD(stack_vec_dVector(uJ), RJ_wrapper, 1e-5));
		};

		cout << "--> dRJduJ" << endl;
		Traj_equality_check(calculate_dRJduJ_FD(uJ), dRduJ); 
	}

	return dRduJ;
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

MatrixNxM SoftLocoSolver::calculate_dxdu(const dVector &u, const dVector &x, const dVector &x_ctc) {
	check_x_size(x);
	check_u_size(u);
	// --
	MatrixNxM dxdtau; // H^{-1} A
	{
		// X H = A_T
		// H_T X_T = A
		SparseMatrix A_s = calculate_A(x);
		SparseMatrix H_T_s = calculate_H(x, u, x_ctc).transpose();
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
	A.resize(DN(), T());
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

SparseMatrix SoftLocoSolver::calculate_H(const dVector &x, const dVector &u, const dVector &x_ctc) {
	check_x_size(x);
	check_u_size(u);
	// --
	vector<MTriplet> triplets;
	{ 
		dVector u_push = mesh->balphac;

		{
			if (!mesh->contacts.empty()) {
				if (x_ctc.size() == 0) { error("[calculate_H] x_ctc not spec'd."); } 
				mesh->update_contacts(x_ctc);
			}

			{
				mesh->balphac = u; // (*)
				auto &E = mesh->energyFunction;
				(SOLVE_DYNAMICS) ? E->setToDynamicsMode(mesh->timeStep) : E->setToStaticsMode(0.);
				E->addHessianEntriesTo(triplets, x);
			}

			if (!mesh->contacts.empty()) {
				mesh->update_contacts(mesh->x);
			}
		}
		mesh->balphac = u_push;

	}
	// H_sparse.setZero(); // (*)
	// H_sparse.setFromTriplets(triplets.begin(), triplets.end());

	// FORNOW; TODO: DupFuntor
	int NUM_TRIPLETS = triplets.size(); // (*)
	for (int i = 0; i < NUM_TRIPLETS; ++i) {
		const auto &triplet = triplets[i];
		if (triplet.row() != triplet.col()) {
			triplets.push_back(MTriplet(triplet.col(), triplet.row(), triplet.value()));
		}
	} 

	SparseMatrix H_sparse(DN(), DN());
	H_sparse.resize(DN(), DN());
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
int SoftLocoSolver::DN() { return D()*N(); } 
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

void SoftLocoSolver::Traj_equality_check(const Traj &T1, const Traj &T2) {
	if (T1.size() != T2.size()) { error("SizeMismatchError"); }
	for (size_t i = 0; i < T1.size(); ++i) {
		cout << "--> k = " << i << " // ";
		vector_equality_check(T1[i], T2[i]);
	}
};

void SoftLocoSolver::MTraj_equality_check(const MTraj &T1, const MTraj &T2) {
	if (T1.size() != T2.size()) { error("SizeMismatchError"); }
	for (size_t i = 0; i < T1.size(); ++i) {
		cout << "--> k = " << i << " // ";
		matrix_equality_check(T1[i], T2[i]);
	}
};

Traj SoftLocoSolver::unstack_Traj(const dVector &fooJ_d) {
	Traj fooJ;
	int LEN = fooJ_d.size() / K;
	for (int i = 0; i < K; ++i) {
		fooJ.push_back(fooJ_d.segment(i*LEN, LEN));
	} 
	return fooJ;
};


// auto vMvD2Traj = [](const vector<MatrixNxM> &vM, const Traj &vD) {
// 	Traj ret;
// 	if (vM.size() != vD.size()) { error("SizeMismatchError"); }
// 	for (size_t i = 0; i < vM.size(); ++i) {
// 		ret.push_back(vM[i] * vD[i]);
// 	}
// 	return ret; 
// }; 
