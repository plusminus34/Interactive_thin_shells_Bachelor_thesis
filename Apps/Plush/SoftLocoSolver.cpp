#include "SoftLocoSolver.h"
#include <PlushHelpers/helpers_star.h>
#include <Eigen/SparseCholesky>
#include "XYPlot.h"

// Rough dev plan:
// Z <- K
// yJ <- uJ
// dudy <- Identity (First hardcode, then check.) 
// // lookup_dudy()

// xJ_of_uJ -> xJ_of_yJ
// calcualte_QJ
// calculate_OJ --> // calculate_gammaJ

// uJ_next --> yJ_next
// calculate_dOduJ

// calculate_RJ
// calculate_dRduJ
// calculate_R, bool loadTendons
// construct_U_barrier_function

SoftLocoSolver::SoftLocoSolver(SimulationMesh *mesh) {

	if ((K-1) % (Z-1) != 0) { error("NotImplementedError"); }

	this->mesh = mesh; 
	xm1_curr = mesh->x;
	vm1_curr = mesh->v;
	resize_zero(um1_curr, T()); // FORNOW

	for (int z = 0; z < Z; ++z) {
		dVector SPLINE_PARAMS_;
		resize_fill(SPLINE_PARAMS_, S(), 0.);// .05 + abs(.1*sin(dfrac(z, Z - 1)*2.*PI)));
		ymJ_curr.push_back(SPLINE_PARAMS_);
	}
	god_spline = new CubicHermiteSpline(vecDouble2dVector(linspace(Z, 0., 1.)), vecDouble2dVector(linspace(K, 0., 1.)));
	dUdY_ = god_spline->calculate_dUdY_();

	god_spline2 = new CubicHermiteSpline_v2(vecDouble2dVector(linspace(Z, 0., 1.)), vecDouble2dVector(linspace(K, 0., 1.)));
	dUdY2_ = god_spline2->calculate_dUdY_();
	dUdM2_ = god_spline2->calculate_dUdM_();

	// for (int _ = 0; _ < K; ++_) { dVector ZERO_; resize_zero(ZERO_, T()); uJ_curr.push_back(ZERO_); }
	// xJ_curr = xJ_of_uJ(uJ_curr); 
	xJ_curr = xJ_of_ymJ(ymJ_curr); 

	for (int _ = 0; _ < K; ++_) { SPEC_FREESTYLE_J.push_back(false); }
	for (int _ = 0; _ < K; ++_) { SPEC_COM_J.push_back(true); }
	for (int i = 0; i < K; ++i) { COMpJ.push_back(mesh->get_COM(xJ_curr[i])); }

	construct_u_barrierFuncs(); 


	COMp_FORNOW = mesh->get_COM(mesh->X); 

	objectiveFunction = new SoftLocoObjectiveFunction(this);
	constraints = new SoftLocoConstraints(this);
	constrainedObjectiveFunction = new ConstrainedObjectiveFunction(objectiveFunction, constraints); 
}

void SoftLocoSolver::draw() { 

	// FORNOW
	// for (int t = 0; t < T(); ++t) {
	// 	for (int z = 0; z < Z; ++z) {
	// 		yJ_curr[z][t] = uJ_curr[z][t] / mesh->balphaz[t];
	// 	} 
	// }

	// int i = -1;	for (auto &Y : zipunzip(yJ_curr)) { god_spline->SPEC_COLOR = kelly_color(++i); 
	// 	god_spline->draw(Y);
	// }
 
	glMasterPush(); {
		glDisable(GL_STENCIL_TEST);
		glDisable(GL_DEPTH_TEST);
		// glTranslated(1., 0., 0.);

		for (int i = K - 1; i >= 0; --i) {
			set_color(color_swirl(dfrac(i, K - 1), ORCHID, RATIONALITY));
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
		}


		for (int i = K - 1; i >= 0; --i) {
			glPointSize(10);
			// if (i != K - 1 && i != K/2) { continue; } // !!!
			if (i != K - 1) { continue; } // !!!

			double RAD = .015;
			set_color(WHITE);
			for (int _ = 0; _ < 2; ++_) {

				int NUM_SEG = 16;
				P3D s = mesh->get_COM(xJ_curr[i]); // circle
				P3D t = COMpJ[i];                  // point
				glBegin(GL_POLYGON); {
					for (int i = 0; i < NUM_SEG; ++i) {
						double theta = dfrac(i, NUM_SEG) * 2.*PI;
						glP3D(s + RAD*e_theta_2D(theta));
					}
				} glEnd();

				if (!IS_ZERO(V3D(s, t).norm())) {
					V3D o = construct_perp2d(V3D(s, t).normalized());
					glBegin(GL_TRIANGLES); {
						glP3D(t);
						glP3D(s + RAD*o);
						glP3D(s + -RAD*o);
					} glEnd();
				}

				RAD = .01;
				set_color(color_swirl(dfrac(i, K - 1), ORCHID, RATIONALITY)); 
			}

			// for (auto &GL_PRIMITIVE : { GL_POINTS, GL_LINES }) {
			// 	glBegin(GL_PRIMITIVE); {
			// 		glP3D(mesh->get_COM(xJ_curr[i]));
			// 		glP3D(COMpJ[i]);
			// 	} glEnd();
			// }

		}
		
	} glMasterPop(); 

	{
		auto K_range = linspace(K, 0., 1.);
		auto Z_range = linspace(Z, 0., 1.);
		vector<XYPlot*> plots;

		dVector ONE;  resize_fill(ONE, K, 1);
		dVector ZERO; resize_zero(ZERO, K);
		vector<dVector> lines = { -.5*ONE, ZERO, .5*ONE };
		for (auto &line : lines) { plots.push_back(new XYPlot(K_range, dVector2vecDouble(line))); }
		for (auto &plot : plots) { plot->SPEC_COLOR = LIGHT_CLAY; }

		// FORNOW
		auto uJ_curr_ = uJ_of_ymJ(ymJ_curr);
		for (int t = 0; t < T(); ++t) {
			vector<double> u_K; for (int k = 0; k < K; ++k) { u_K.push_back(uJ_curr_[k][t] / mesh->balphaz[t]); }
			// vector<double> y_Z; for (int z = 0; z < Z; ++z) { y_Z.push_back(yJ_curr[z][t] / mesh->balphaz[t]); }

			XYPlot *plot_K = new XYPlot(K_range, u_K);
			plot_K->THIN_LINES = true;
			plot_K->DRAW_POINTS = false;
			plot_K->SPEC_COLOR = kelly_color(t); 
			plots.push_back(plot_K);

			// XYPlot *plot_Z = new XYPlot(Z_range, y_Z);
			// plot_Z->DRAW_LINES = false;
			// plot_Z->SPEC_COLOR = kelly_color(t); 
			// plots.push_back(plot_Z);
		}

		for (auto &plot : plots) {
			*plot->origin = P3D(-2., -1.);
			*plot->top_right = *plot->origin + V3D(1., 2.);
		};

		XYPlot::uniformize_axes(plots);
		for (auto &plot : plots) { plot->draw(); }
		for (auto &plot : plots) { delete plot; }
	}

	// { auto K_range = linspace(K, 0., 1.);
	// 	auto Z_range = linspace(Z, 0., 1.);
	// 	vector<XYPlot*> plots;

	// 	// dVector POINT_ONE;  resize_fill(POINT_ONE, K, .1);
	// 	dVector ZERO; resize_zero(ZERO, K);
	// 	// vector<dVector> lines = { -POINT_ONE, ZERO, POINT_ONE };
	// 	vector<dVector> lines = { ZERO };
	// 	for (auto &line : lines) { plots.push_back(new XYPlot(K_range, dVector2vecDouble(line))); }
	// 	for (auto &plot : plots) { plot->SPEC_COLOR = WHITE; plot->DRAW_POINTS = false; plot->THIN_LINES = true; }

	// 	// FORNOW
	// 	auto uJ_curr_ = uJ_of_yJ(yJ_curr);
	// 	for (int t = 0; t < T(); ++t) {
	// 		vector<double> gamma_K; for (int k = 0; k < K; ++k) { gamma_K.push_back(mesh->tendons[t]->get_Gamma(xJ_curr[k], uJ_curr_[k])); };

	// 		XYPlot *plot_K = new XYPlot(K_range, gamma_K);
	// 		plot_K->THIN_LINES = true;
	// 		plot_K->DRAW_POINTS = false;
	// 		plot_K->SPEC_COLOR = kelly_color(t); 
	// 		plots.push_back(plot_K);

	// 	}


	// 	for (int t = 0; t < T(); ++t) {
	// 		vector<double> gamma_Z; for (int z = 0; z < Z; ++z) {
	// 			int k = k_of_z(z);
	// 			gamma_Z.push_back(mesh->tendons[t]->get_Gamma(xJ_curr[k], uJ_curr_[k]));
	// 		};

	// 		XYPlot *plot_Z = new XYPlot(Z_range, gamma_Z);
	// 		plot_Z->DRAW_LINES = false;
	// 		plot_Z->DRAW_POINTS = true;
	// 		plot_Z->SPEC_COLOR = kelly_color(t); 
	// 		plots.push_back(plot_Z);

	// 	}

	// 	for (auto &plot : plots) {
	// 		*plot->origin = P3D(-2.75, -1.);
	// 		*plot->top_right = *plot->origin + V3D(1., 2.);
	// 	};

	// 	XYPlot::uniformize_axes(plots);
	// 	for (auto &plot : plots) { plot->draw(); }
	// 	for (auto &plot : plots) { delete plot; }
	// }

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


	// if (mesh->UNILATERAL_TENDONS &&PROJECT) { projectJ(); } 

	// {
	// 	for (int _ = 0; _ < NUM_ITERS_PER_STEP; ++_) {
	// 		iterate();
	// 	}
	// }

	{
		// // // BFGSFunctionMinimizer minimizer(5);
		// GradientDescentFunctionMinimizer minimizer(1);
		// minimizer.solveResidual = 1e-5;
		// minimizer.maxLineSearchIterations = 15;
		// minimizer.lineSearchStartValue = 1.;
		// minimizer.printOutput = true;
		// // --
		// dVector yS = stack_vec_dVector(yJ_curr);
		// double functionValue = objectiveFunction->computeValue(yS);
		// minimizer.minimize(objectiveFunction, yS, functionValue);
	}

	{
		SQPFunctionMinimizer minimizer(1);
		minimizer.maxLineSearchIterations_ = 15;
		minimizer.printOutput_ = false;
		// --
		dVector ymS = stack_vec_dVector(ymJ_curr);
		double functionValue = objectiveFunction->computeValue(ymS);
		minimizer.minimize(constrainedObjectiveFunction, ymS, functionValue); 
	}

	// cout << " O  = " << calculate_OJ(yJ_curr) << endl;
}

void SoftLocoSolver::iterate() {
	error("iterate():DeprecatedError");
	// // uJ_curr = uJ_next(uJ_curr, xJ_curr);
	// // xJ_curr = xJ_of_uJ(uJ_curr);
	// yJ_curr = yJ_next(yJ_curr, xJ_curr);
	// xJ_curr = xJ_of_yJ(yJ_curr);
}

Traj SoftLocoSolver::xZ_from_xJ(const Traj &xJ) {
	// TODO: Rename *J -> *K
	Traj xZ;
	for (int z = 0; z < Z; ++z) {
		int k = k_of_z(z);
		xZ.push_back(xJ[k]);
	}
	return xZ;
}

// void SoftLocoSolver::projectJ() { 

// 	// Approach 1: Ensure all tendons taught in all _keyframes_ 

// 	Traj x_tmp;
// 	Traj v_tmp;

// 	// Traj xZ_tmp = xZ_from_xJ(xJ_curr);

// 	for (int z = 0; z < Z; ++z) {
// 		int k = k_of_z(z);

// 		const dVector &x_start = (x_tmp.empty()) ? xm1_curr : x_tmp.back();
// 		const dVector &v_start = (v_tmp.empty()) ? vm1_curr : v_tmp.back();
// 		// --
// 		dVector &y_proj = yJ_curr[z]; // (*)
// 		Traj uJ_proj = uJ_of_yJ(yJ_curr);
// 		dVector u_proj = uJ_proj[k]; // TODO: RENAME uK_of_yZ
// 		while (true) {
// 			bool PROJECTED = false;

// 			auto xv = mesh->solve_dynamics(x_start, v_start, u_proj);
// 			dVector x_proj = xv.first;

// 			for (int i = 0; i < T(); ++i) {
// 				double Gamma_i;
// 				Gamma_i = mesh->tendons[i]->get_Gamma(x_proj, u_proj);

// 				if (Gamma_i < .0005) {
// 					PROJECTED = true;
// 					y_proj[i] += .001 - Gamma_i; // yJ_curr[z];
// 					uJ_proj = uJ_of_yJ(yJ_curr);
// 					u_proj = uJ_proj[k]; // TODO: RENAME uK_of_yZ 
// 				}
// 			}

// 			if (!PROJECTED) {
// 				break;
// 			}
// 		}
// 		yJ_curr[z] = y_proj;

// 		// Integrate forward into [x|v]_start
// 		if (z != Z - 1) {
// 			for (int step = 0; step < (K - 1) / (Z - 1); ++step) {
// 				const dVector &x_start = (x_tmp.empty()) ? xm1_curr : x_tmp.back();
// 				const dVector &v_start = (v_tmp.empty()) ? vm1_curr : v_tmp.back();
// 				// --
// 				auto xv = mesh->solve_dynamics(x_start, v_start, uJ_proj[k + step]);
// 				x_tmp.push_back(xv.first);
// 				v_tmp.push_back(xv.second);
// 			}
// 		}
// 	}

// 	xJ_curr = xJ_of_yJ(yJ_curr); 

// }

// Traj SoftLocoSolver::uJ_next(const Traj &uJ, const Traj &xJ) { 
// 	vector<dRowVector> dOduJ = calculate_dOduJ(uJ, xJ);
// 	double gammaJ = calculate_gammaJ(uJ, dOduJ);
// 	Traj uJ_next;
// 	for (int i = 0; i < K; ++i) {
// 		uJ_next.push_back(uJ[i] - gammaJ * dOduJ[i].transpose());
// 	}
// 	return uJ_next;
// }

// Traj SoftLocoSolver::yJ_next(const Traj &yJ, const Traj &xJ) { 
// 	vector<dRowVector> dOdyJ = calculate_dOdyJ(yJ, xJ);
// 	// dOdyJ[0].setZero();
// 	double gammaJ = calculate_gammaJ(yJ, dOdyJ);
// 	Traj yJ_next;
// 	for (int i = 0; i < Z; ++i) {
// 		yJ_next.push_back(yJ[i] - gammaJ * dOdyJ[i].transpose());
// 	}
// 	return yJ_next;
// }
//  
// double SoftLocoSolver::calculate_gammaJ(const Traj &yJ, const vector<dRowVector> &dOdyJ) {
// 	double gammaJ_0 = 1.;// .1 / stack_vec_dRowVector(dOdyJ).squaredNorm();
// 	int maxLineSearchIterations = 24;

// 	Traj yJ_0 = yJ;
// 	double O_0 = calculate_OJ(yJ_0); 
// 	double gammaJ_k = gammaJ_0;
// 	for (int j = 0; j < maxLineSearchIterations; j++) { 
// 		Traj yJ_k; // = yJ_0 - gammaJ_k * dOdyJ; 
// 		for (int i = 0; i < Z; ++i) {
// 			yJ_k.push_back(yJ_0[i] - gammaJ_k * dOdyJ[i].transpose());
// 		}
// 		double O_k = calculate_OJ(yJ_k);

// 		if (!isfinite(O_k)) {
// 			error("non-finite o_k detected.");
// 			O_k = O_0 + 1.0;
// 		}

// 		if (O_k > O_0) {
// 			gammaJ_k /= 2.0;
// 		} else {
// 			// success
// 			return gammaJ_k;
// 		}
// 	}

// 	error("SoftLoco line search failed.");
// 	return gammaJ_k; 
// }

// -- //
 
Traj SoftLocoSolver::xJ_of_ymJ(const Traj &ymJ) {
	return xJ_of_uJ(uJ_of_ymJ(ymJ));
}

Traj SoftLocoSolver::xJ_of_uJ(const Traj &uJ) {
	return solve_trajectory(mesh->timeStep, xm1_curr, vm1_curr, uJ);
}

Traj SoftLocoSolver::uJ_of_ymJ(const Traj &ymJ) {
	Traj uJ_zzz;
	Traj ymJ_zzz = zipunzip(ymJ);
	for (int i = 0; i < T(); ++i) { // _for_tendon_i 
		uJ_zzz.push_back(god_spline2->calculate_U(ymJ_zzz[i], ymJ_zzz[T() + i]));
	}
	return zipunzip(uJ_zzz);
}

// -- //

void SoftLocoSolver::FD_TEST_dOJdymJ(const Traj &ymJ, const Traj &xJ) {
	auto calculate_dOJdymJ_FD = [&](const Traj &ymJ) -> Traj {
		auto OJ_wrapper = [&](const dVector &ymJ_d) -> double {
			return calculate_OJ(unstack_Traj(ymJ_d, Z)); 
		};
		return unstack_Traj(vec_FD(stack_vec_dVector(ymJ), OJ_wrapper, FD_TEST_STEPSIZE), Z);
	};

	cout << "--> dOJdyJ" << endl;
	// NOTE: anal version also changes when we run HP_NEWTON
	Traj_equality_check(calculate_dOJdymJ_FD(ymJ), calculate_dOdymJ(ymJ, xJ)); 
}

double SoftLocoSolver::calculate_OJ(const Traj &ymJ) {
	auto uJ_ = uJ_of_ymJ(ymJ);
	double QJ = calculate_QJ(uJ_);
	double RJ = calculate_RJ(uJ_);
	return QJ + RJ;
	// return QJ;
}

double SoftLocoSolver::calculate_QJ(const Traj &uJ) {

	Traj xJ;
	if (!LINEAR_APPROX) {
		xJ = xJ_of_uJ(uJ);
	} else { 
		error("NotImplmenetedError.");
		// Traj duJ;
		// for (int i = 0; i < K; ++i) {
		// 	duJ.push_back(uJ[i] - uJ_curr[i]);
		// }

		// // CHECKME
		// for (int i = 0; i < K; ++i) {
		// 	MatrixNxM entry = xJ_curr[i];
		// 	for (int j = 0; j < K; ++j) {
		// 		entry += dxiduj_SAVED[i][j] * duJ[j];
		// 	}
		// 	xJ.push_back(entry);
		// } 
	}

	double QJ = 0.;
	for (int i = 0; i < K; ++i) {
		// if (i != 0) { continue; } // !!!
		// if (i != K - 1 && i != K/2) { continue; } // !!!
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

	// RJ += 1e5*.5*uJ[0].squaredNorm();

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

	// {
	// 	for (int i = 0; i < T(); ++i) {
	// 		ret += u_barrierFuncs[i]->computeValue(u[i]);
	// 	}
	// }

	if (REGULARIZE_u) {
		for (int i = 0; i < T(); ++i) {
			ret += u_regFunc->computeValue(u[i]);
		}
	}

	return ret; 
}

vector<dRowVector> SoftLocoSolver::calculate_dOdymJ(const Traj &ymJ, const Traj &xJ) {
	auto uJ_ = uJ_of_ymJ(ymJ);
	vector<dRowVector> dQdyJ = dSTARduJ2dSTARdymJ(calculate_dQduJ(uJ_, xJ)); 
	vector<dRowVector> dRdyJ = dSTARduJ2dSTARdymJ(calculate_dRduJ(uJ_)); 
	vector<dRowVector> dOdyJ;
	for (int i = 0; i < Z; ++i) {
		dOdyJ.push_back(dQdyJ[i] + dRdyJ[i]);
	}
	return dOdyJ;
	// return dQdyJ;
}

vector<dRowVector> SoftLocoSolver::calculate_dQduJ(const Traj &uJ, const Traj &xJ) {

	auto calculate_dQJduJ_FD = [&](const Traj &uJ, const Traj &xJ) -> Traj {
		auto QJ_wrapper = [&](const dVector uJ_d) -> double {
			return calculate_QJ(unstack_Traj(uJ_d, K)); 
		};
		return unstack_Traj(vec_FD(stack_vec_dVector(uJ), QJ_wrapper, 1e-8), K);
	};
 
	vector<dVector> dQkdxk_FD; 
	vector<MatrixNxM> dxkdxkm1_FD; 
	vector<vector<MatrixNxM>> dxidxj_FD; 
	vector<MatrixNxM> dxkduk_FD; 
	if (TEST_Q_FD) {
		if (true) {
			for (int k = 0; k < K; ++k) {
				dVector entry; resize_zero(entry, DN());
				// if (k == K - 1 || k == K/2) {
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
	// SparseMatrix M = SparseMatrix(mesh->m.asDiagonal());
	// SparsMatrix f1h2M = (1./(h*h))*M;
	SparseMatrix M = SparseMatrix(mesh->m.asDiagonal());
	SparseMatrix f1h2M = (1./(h*h))*M;
	// vector<MatrixNxM> vecHinv;
	// for (int k = 0; k < K; ++k) {
	// 	dVector x_ctc = (k == 0) ? xm1_curr : xJ[k - 1];
	// 	vecHinv.push_back(calculate_H(xJ[k], uJ[k], x_ctc).toDense().inverse());
	// }
	vector<SparseMatrix> vecH;
	for (int k = 0; k < K; ++k) {
		dVector x_ctc = (k == 0) ? xm1_curr : xJ[k - 1];
		vecH.push_back(calculate_H(xJ[k], uJ[k], x_ctc));
	}

	// cout << "dxkdxkm1" << endl;
	// vector<SparseMatrix> dGkdxkm1_INDEX_at_km1;
	vector<SparseMatrix> dScriptGkdxkm1_INDEX_at_km1;
	vector<SparseMatrix> dxkdxkm1_INDEX_AT_km1; {
		for (int k = 1; k < K; ++k) {
			auto &H = vecH[k];

			// MatrixNxM dGkdxkm1(DN(), DN());
			SparseMatrix dScriptGkdxkm1(DN(), DN());
			const dVector &xk   = xJ[k];
			const dVector &xkm1 = xJ[k - 1];
			mesh->update_contacts(xkm1); // FORNOW

			sparse_mat_resize_zero(dScriptGkdxkm1, DN(), DN());
			// mat_resize_zero(dGkdxkm1, DN(), DN());
			for (auto &c: mesh->contacts) {
				int i = c->node->nodeIndex;
				P3D xy_k_i   = c->getPosition(xk);
				P3D xy_km1_i = c->getPosition(xkm1);
				double &xk_i   = xy_k_i[0];
				double &xkm1_i = xy_km1_i[0];
				dScriptGkdxkm1.insert(2*i, 2*i)     = -c->b()      *c->QT->computeSecondDerivative(xk_i - xkm1_i);
				dScriptGkdxkm1.insert(2*i, 2*i + 1) =  c->b_prime()*c->QT->computeDerivative(      xk_i - xkm1_i);
			}
			dScriptGkdxkm1 -= 2.*f1h2M;
			dScriptGkdxkm1_INDEX_at_km1.push_back(dScriptGkdxkm1);

			// if (TEST_Q_FD) {
			// 	MatrixNxM dGkdxkm1_FD;
			// 	auto G_wrapper = [&](const dVector xkm1) -> dVector {
			// 		dVector G;
			// 		mesh->update_contacts(xkm1);
			// 		mesh->energyFunction->setToStaticsMode(0.);
			// 		mesh->energyFunction->addGradientTo(G, xk);
			// 		(SOLVE_DYNAMICS) ? mesh->energyFunction->setToDynamicsMode(mesh->timeStep) : mesh->energyFunction->setToStaticsMode(0.);
			// 		return G;
			// 	};
			// 	dGkdxkm1_FD = mat_FD(xkm1, G_wrapper, 5e-5);

			// 	cout << "--> dGkdxkm1" << endl;
			// 	matrix_equality_check(dGkdxkm1_FD, dGkdxkm1);
			// }

			// SparseMatrix dScriptGkdxkm1 = dGkdxkm1 - 2.*f1h2M;
			// // MatrixNxM dScriptGdxkm1 = -2.*f1h2M; // FORNOW
			// dScriptGkdxkm1_INDEX_at_km1.push_back(dScriptGkdxkm1);

			// TOOD: These two should be brought into the general case.
			// Make sure gradient checks with FD before WIHTOUT FACT
			// dxkdxkm1_INDEX_AT_km1.push_back(solve_AX_EQUALS_B(-H, dScriptGkdxkm1));
			// dxkdxkm1_INDEX_AT_km1.push_back(solve_AX_EQUALS_B_WITHOUT_FACTORIZATION(-H, dScriptGkdxkm1));
		}
		mesh->update_contacts(mesh->x);
	} 

	// // cout << "dxkdxkm2" << endl;
	// vector<SparseMatrix> dxkdxkm2_INDEX_AT_km2; {
	// 	for (int k = 2; k < K; ++k) {
	// 		double h = mesh->timeStep;
	// 		auto &H = vecH[k];
	// 		SparseMatrix TERM_1 = dScriptGkdxkm1_INDEX_at_km1[k - 1] * dxkdxkm1_INDEX_AT_km1[k - 2];
	// 		SparseMatrix &TERM_2 = f1h2M;
	// 		dxkdxkm2_INDEX_AT_km2.push_back(solve_AX_EQUALS_B(-H, TERM_1 + TERM_2));
	// 	}
	// } 
 
	// cout << "dxidxj" << endl;
	
	// -- // Seed.
	// auto &D1 = [&](int k) { return dxkdxkm1_INDEX_AT_km1[k - 1]; };
	// auto &D2 = [&](int k) { return dxkdxkm2_INDEX_AT_km2[k - 2]; };
	// --
	vector<vector<SparseMatrix>> dxidxj;
	for (int i = 0; i < K; ++i) {
		vector<SparseMatrix> row;
		for (int j = 0; j < K; ++j) {
			int imj = i - j;
			// --
			SparseMatrix entry;
			// --
			if (imj < 0) {
				entry.resize(DN(), DN()); entry.setZero();
			} else if (imj == 0) {
				entry.resize(DN(), DN()); entry.setIdentity();
			} else {
				entry.resize(DN(), DN()); entry.setZero();
			// } if (imj == 1) {
			// 	entry = D1(i);
			// } else if (imj == 2) {
			// 	entry = D2(i);
			}
			row.push_back(entry);
		}
		dxidxj.push_back(row);
	} 

	SparseMatrix ZERO(DN(), DN()); ZERO.setZero();
	auto &XX = [&](int i, int j) -> const SparseMatrix & {
		if (i < 0) { return ZERO; }
		return dxidxj[i][j];
	};

	// Grow
	// TODO (SAFER): One solver per Hi, prefactorized vecHSolvers_FACTORIZED
	analyze_A(solver, vecH[0]);
	auto &scrGxm1 = [&](int k) { return dScriptGkdxkm1_INDEX_at_km1[k - 1]; };
	for (int j = 0; j < K; ++j) {
		for (int i = j + 1; i < K; ++i) { 
			auto &Hi = vecH[i];
			dxidxj[i][j] = solve_AX_EQUALS_B_WITHOUT_ANALYSIS(solver, -Hi, scrGxm1(i)*XX(i - 1, j) + f1h2M*XX(i - 2, j));
		}
	}

	// cout << "dxkduk" << endl;
	vector<SparseMatrix> dxkduk;
	for (int k = 0; k < K; ++k) {
		dVector x_ctc = (k == 0) ? xm1_curr : xJ[k - 1];
		dxkduk.push_back(calculate_dxdu(uJ[k], xJ[k], x_ctc));
	}
	
	// cout << "dQkdxk" << endl;
	vector<dRowVector> dQJdxk;
	for (int k = 0; k < K; ++k) {
		dRowVector entry;
		// if (k != K - 1 && k != K/2) {
		if (k != K - 1) {
			entry.setZero(DN());
		} else {
			entry = calculate_dQdx(xJ[k], COMpJ[k]);
		}
		dQJdxk.push_back(entry);
	}

	// cout << "dQJduj" << endl;
	vector<dRowVector> dQJduj; // (*)
	for (int j = 0; j < K; ++j) {
		dRowVector entry; resize_zero(entry, T());
		for (int i = j; i < K; ++i) { // (*)
			entry += dQJdxk[i] * dxidxj[i][j] * dxkduk[j];
		}
		dQJduj.push_back(entry);
	}

	// if (TEST_Q_FD) {
	// 	cout << "BEG.................................................." << endl;
	// 	cout << "--> dxkduk" << endl;
	// 	MTraj_equality_check(dxkduk_FD, dxkduk);
	// 	// cout << "--> dxkdxkm1" << endl;
	// 	// MTraj_equality_check(dxkdxkm1_FD, dxkdxkm1_INDEX_AT_km1);
	// 	cout << "--> dxidxj" << endl;
	// 	for (int i = 0; i < K; ++i) {
	// 		cout << "----> ii = " << i << endl;
	// 		vector<vector<MatrixNxM>> dxidxj_dense;
	// 		for (auto &row : dxidxj) {
	// 			vector<MatrixNxM> dense_row;
	// 			for (auto &entry : row) {
	// 				dense_row.push_back(entry.toDense());
	// 			}
	// 			dxidxj_dense.push_back(dense_row);
	// 		}
	// 		MTraj_equality_check(dxidxj_FD[i], dxidxj_dense[i]);
	// 	}
	// 	cout << "--> dQkdxk" << endl;
	// 	Traj_equality_check(dQkdxk_FD, dQJdxk);
	// 	cout << "..................................................END" << endl;
	// }
 
	if (TEST_Q_FD) {
		cout << "--> dQJduJ" << endl;
		Traj STEP0 = calculate_dQJduJ_FD(uJ, xJ);
		Traj_equality_check(STEP0, dQJduj);
	}

	// dxiduj_SAVED.clear();
	// for (int i = 0; i < K; ++i) {
	// 	vector<MatrixNxM> row;
	// 	for (int j = 0; j < K; ++j) {
	// 		row.push_back(dxidxj[make_pair(i, j)] * dxkduk[j]);
	// 	}
	// 	dxiduj_SAVED.push_back(row);;
	// }

	return dQJduj;
}

vector<dRowVector> SoftLocoSolver::dSTARduJ2dSTARdymJ(const vector<dRowVector> &dSTARdui) {
	SparseMatrix dUdY_ = god_spline2->calculate_dUdY_();
	SparseMatrix dUdM_ = god_spline2->calculate_dUdM_();
	vector<dRowVector> dSTARdymj;
	for (int j = 0; j < Z; ++j) {
		dRowVector entry; entry.setZero(S());
		for (int i = 0; i < K; ++i) {
			entry.head(T()) += dSTARdui[i] * dUdY_.coeff(i, j);
			entry.tail(T()) += dSTARdui[i] * dUdM_.coeff(i, j);
		}
		dSTARdymj.push_back(entry);
	}
	return dSTARdymj;
}

vector<dRowVector> SoftLocoSolver::calculate_dRduJ(const Traj &uJ) {
	vector<dRowVector> dRduJ;

	for (auto &u : uJ) {
		dRduJ.push_back(calculate_dRdu(u));
	}

	// dRduJ[0] += 1e5*uJ[0].transpose();
 
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
				return calculate_RJ(unstack_Traj(uJ_d, K)); 
			};
			return unstack_Traj(vec_FD(stack_vec_dVector(uJ), RJ_wrapper, 1e-5), K);
		};

		cout << "--> dRJduJ" << endl;
		Traj_equality_check(calculate_dRJduJ_FD(uJ), dRduJ); 
	}

	return dRduJ;
}

dRowVector SoftLocoSolver::calculate_dQdx(const dVector &x, const P3D &COMp) { 
	check_x_size(x);
	// --
	dRowVector dQdx;
	{
		V3D DeltaCOM_ = V3D(COMp, mesh->get_COM(x));
		dRowVector dQdCOM; dQdCOM.setZero(D()); // dRowVector dQdCOM = DeltaCOM;
		for (int i = 0; i < D(); ++i) { dQdCOM[i] = DeltaCOM_[i]; }
		// --
		dQdx = dQdCOM * mesh->get_dCOMdx();
	}

	return dQdx;
}

SparseMatrix SoftLocoSolver::solve_AX_EQUALS_B(const SparseMatrix &A, const SparseMatrix &B) {
	Eigen::SimplicialLDLT<SparseMatrix> ldlt;
	ldlt.compute(A); // TODO: Can split and then one half you only need to do when sparsity structure changes.
	if (ldlt.info() != Eigen::Success) { error("Eigen::SimplicialLDLT decomposition failed."); } 
	SparseMatrix X = ldlt.solve(B);
	if (ldlt.info() != Eigen::Success) { error("Eigen::SimplicialLDLT solve failed."); }
	return X;
}

void SoftLocoSolver::analyze_A(SLSSolver &solver, const SparseMatrix &A) {
	solver.analyzePattern(A);
}

SparseMatrix SoftLocoSolver::solve_AX_EQUALS_B_WITHOUT_ANALYSIS(SLSSolver &solver, const SparseMatrix &A, const SparseMatrix &B) { 
	solver.factorize(A);
	SparseMatrix X = solver.solve(B);
	if (solver.info() != Eigen::Success) { error("Eigen::SimplicialLDLT solve failed."); }
	return X;
}

SparseMatrix SoftLocoSolver::calculate_dxdu(const dVector &u, const dVector &x, const dVector &x_ctc) {
	check_x_size(x);
	check_u_size(u);
	// --
	SparseMatrix dxdtau; // H^{-1} A
	{
		SparseMatrix A = calculate_A(x);
		SparseMatrix H = calculate_H(x, u, x_ctc);
		dxdtau = solve_AX_EQUALS_B(H, A);
	}

	dVector dtaudu_diag; // == dVector dtaudGamma_diag;
	{
		resize_zero(dtaudu_diag, T());
		for (int j = 0; j < T(); ++j) {
			auto &tendon = mesh->tendons[j];
			auto E_j = tendon->tendon_energy_model();
			double Gamma_j = tendon->get_Gamma(x, u);
			dtaudu_diag[j] = E_j->computeSecondDerivative(Gamma_j);
		}
	}
 
	return dxdtau * dtaudu_diag.asDiagonal();
}

dRowVector SoftLocoSolver::calculate_dRdu(const dVector &u) { 
	check_u_size(u);
	// --
	dRowVector dRdu; dRdu.setZero(T());

	// {
	// 	dRowVector duBarrierdu; duBarrierdu.setZero(T());
	// 	for (int i = 0; i < T(); ++i) {
	// 		duBarrierdu[i] += u_barrierFuncs[i]->computeDerivative(u[i]);
	// 	}
	// 	dRdu += duBarrierdu;
	// }

	if (REGULARIZE_u) { 
		dRowVector duRegdu; duRegdu.setZero(T());
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
		double ALPHAC_MAX = .5*tendon->get_alphaz(); // TODO: Make this a parameter
		u_barrierFuncs.push_back(new ZeroCubicQuadratic(1e5, .01, V3D(ALPHAC_MAX, 0.), false, false));
	}
}


////////////////////////////////////////////////////////////////////////////////

int SoftLocoSolver::D() { return mesh->D(); } 
int SoftLocoSolver::N() { return mesh->N(); } 
int SoftLocoSolver::DN() { return D()*N(); } 
int SoftLocoSolver::T() { return mesh->tendons.size(); }
int SoftLocoSolver::S() { return 2*T(); } // Spline params
int SoftLocoSolver::ZS() { return Z*S(); }

////////////////////////////////////////////////////////////////////////////////

bool SoftLocoSolver::check_x_size(const dVector &x) {
	if (x.size() != D() * N()) {
		error("x:InputDimensionsError");
		return false;
	}
	return true;
}
bool SoftLocoSolver::check_u_size(const dVector &u) {
	if (u.size() != T()) {
		error("u:InputDimensionsError");
		return false;
	}
	return true;
}

bool SoftLocoSolver::check_y_size(const dVector &y) {
	if (y.size() != Z) {
		error("y:InputDimensionsError");
		return false;
	}
	return true;
}

// -- //

void SoftLocoSolver::Traj_equality_check(const Traj &T1, const vector<dRowVector> &T2) {
	if (T1.size() != T2.size()) { error("SizeMismatchError"); }
	for (size_t i = 0; i < T1.size(); ++i) {
		cout << "--> k = " << i << " // ";
		vector_equality_check(T1[i], T2[i].transpose());
	}
};

void SoftLocoSolver::Traj_equality_check(const Traj &T1, const Traj &T2) {
	if (T1.size() != T2.size()) { error("SizeMismatchError"); }
	for (size_t i = 0; i < T1.size(); ++i) {
		cout << "--> k = " << i << " // ";
		vector_equality_check(T1[i], T2[i]);
	}
};

void SoftLocoSolver::MTraj_equality_check(const MTraj &T1, const vector<SparseMatrix> &T2) {
	if (T1.size() != T2.size()) { error("SizeMismatchError"); }
	for (size_t i = 0; i < T1.size(); ++i) {
		cout << "--> k = " << i << " // ";
		matrix_equality_check(T1[i], T2[i].toDense());
	}
};

void SoftLocoSolver::MTraj_equality_check(const MTraj &T1, const MTraj &T2) {
	if (T1.size() != T2.size()) { error("SizeMismatchError"); }
	for (size_t i = 0; i < T1.size(); ++i) {
		cout << "--> k = " << i << " // ";
		matrix_equality_check(T1[i], T2[i]);
	}
};

Traj SoftLocoSolver::unstack_Traj(const dVector &fooJ_d, int VEC_SIZE) {
	Traj fooJ;
	int LEN = fooJ_d.size() / VEC_SIZE;
	for (int i = 0; i < VEC_SIZE; ++i) {
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

// cout << matRCstr(dQkdxk[i]);
// cout << matRCstr(dxidxj[i][j]);
// cout << matRCstr(dxkduk[j]);
// cout << endl;

	// TODO: This may actually end up being quite a non-trivial function to write.


	// Traj x_tmp;
	// Traj v_tmp;
	// for (int i = 0; i < K; ++i) {
	// 	const dVector x_start = (x_tmp.empty()) ? xm1_curr : x_tmp.back();
	// 	const dVector v_start = (v_tmp.empty()) ? vm1_curr : v_tmp.back();
	// 	// --
	// 	dVector u_proj = uJ_curr[i];
	// 	while (true) {
	// 		bool PROJECTED = false;

	// 		auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(x_start, v_start, u_proj) : mesh->solve_statics(x_start, u_proj);
	// 		dVector x_proj = xv.first;

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
	// 	auto xv = (SOLVE_DYNAMICS) ? mesh->solve_dynamics(x_start, v_start, u_proj) : mesh->solve_statics(x_start, u_proj);
	// 	x_tmp.push_back(xv.first);
	// 	v_tmp.push_back(xv.second);
	// 	uJ_curr[i] = u_proj;
	// 	xJ_curr[i] = xv.first;
	// }

	// // if (false) { // VALIDATE_PROJECTION
	// // 	Traj xJ_CHECK = solve_trajectory(mesh->timeStep, xm1_curr, vm1_curr, uJ_curr);
	// // 	cout << "^^^" << endl;
	// // 	Traj_equality_check(xJ_curr, xJ_CHECK);
	// // 	cout << "vvv" << endl;
	// // }
