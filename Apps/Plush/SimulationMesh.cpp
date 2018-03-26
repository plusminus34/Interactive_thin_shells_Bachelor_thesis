#include "SimulationMesh.h"

#include <OptimizationLib/NewtonFunctionMinimizer.h>
#include <OptimizationLib/GradientDescentFunctionMinimizer.h>
#include <OptimizationLib/BFGSFunctionMinimizer.h>
#include <OptimizationLib/CMAFunctionMinimizer.h>

#include <GUILib/GLUtils.h>
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>

#include "CSTElement2D.h"
#include "CSTElement3D.h"
#include "CSTSimulationMesh2D.h"
#include "Tendon.h"

#include <PlushHelpers/helpers_star.h>


SimulationMesh::SimulationMesh() {
	energyFunction = NULL;
	checkDerivatives = false;
	checkGradients = false;
	checkHessians = false;
}

SimulationMesh::~SimulationMesh() {
	delete energyFunction;
}

void SimulationMesh::draw(dVector &x, dVector &alphac) {

	if (x.size() == 0)      { x      = this->x;       }
	if (alphac.size() == 0) { alphac = this->balphac; }

	// https://stackoverflow.com/questions/3388294/opengl-question-about-the-usage-of-gldepthmask 
	// // All opaque 
	// --
	if (DRAW_ALL_ELEMENTS_OVERRIDE) {
		for (auto &element       : elements      ) { element->draw(x);       }
		for (auto &non_E_element : non_E_elements) { non_E_element->draw(x); }
	} else {
		if (DRAW_SIMPLICES) { for (auto &simplex : simplices           ) { simplex->draw(x);       } }
		// TODO: if (DRAW_TENDONS)   { for (auto &tendon : tendons              ) { tendon->draw(x, balphac);        } }
		if (DRAW_TENDONS)   { for (auto &tendon : tendons              ) { tendon->draw(x, alphac);        } }
		if (DRAW_PINS)      { for (auto &pin : pins                    ) { pin->draw(x);           } }
		if (DRAW_ANCHORS)   { for (auto &contact : contacts  ) { contact->draw(x);  } }
		if (DRAW_NON_E)     { for (auto &non_E_element : non_E_elements) { non_E_element->draw(x); } }
	}
	// -- 
	if (DRAW_NODAL_FORCES) { drawNodalForces(); }
	if (DRAW_NODAL_VELOCITIES) { drawNodalVelocities(); }

	// // All transparent 
	// -- 
	if (DRAW_BOUNDARY) {
		// if (D() == 3) { glDepthMask(GL_FALSE); }
		for (auto &boundary_simplex : boundary_simplices) {
			boundary_simplex->draw(x);
		}
		// if (D() == 3) { glDepthMask(GL_TRUE); }
	}

}

void SimulationMesh::applyYoungsModulusAndPoissonsRatio(const double &youngsModulus, const double &poissonsRatio) {
	for (auto &simplex : simplices) {
		if (D() == 2) {
			dynamic_cast<CSTElement2D *>(simplex)->setYoungsModulusAndPoissonsRatio(dynamic_cast<CSTElement2D *>(simplex)->W*youngsModulus, poissonsRatio); // TODO: THICKNESS
		} else if (D() == 3) {
			dynamic_cast<CSTElement3D *>(simplex)->setYoungsModulusAndPoissonsRatio(youngsModulus, poissonsRatio); 
		}
	}

}

double SimulationMesh::get_totalMass() {
	double ret = 0;
	P3D NUM = P3D();
	for (auto &node : this->nodes) {
		double mass = this->m[node->nodeIndex];
		ret += mass;
	}
	return ret; 
}

P3D SimulationMesh::get_COM(const dVector &y) { 
	P3D NUM = P3D();
	for (auto &node : this->nodes) {
		P3D r = node->getCoordinates(y);
		double m = node->getMass();
		NUM += r*m;
	}
	double DEN = get_totalMass(); 
	return NUM / DEN;
}

MatrixNxM SimulationMesh::get_dCOMdx() { 
	MatrixNxM NUM; mat_resize_zero(NUM, D(), D()*N());
	for (size_t block = 0; block < nodes.size(); ++block) {
		Node *node = nodes[block];
		double m = node->getMass();
		for (int d = 0; d < D(); ++d) {
			int col_offset = D()*block;
			NUM(d, d + col_offset) = m;
		}
	}
	double DEN = get_totalMass(); 
	return NUM / DEN;
}

////////////////////////////////////////////////////////////////////////////////
// core pieces /////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int SimulationMesh::N() {
	return nodes.size();
}
 
int SimulationMesh::C() {
	return contacts.size();
}

void SimulationMesh::prep_nodal_properties_dVectors() {
	int DN = D()*N();
	resize_zero(X,       DN);
	resize_zero(x,       DN);
	resize_zero(x_prime, DN);
	resize_zero(v,       DN);
	resize_zero(f_ext,   DN);
	resize_zero(f_ctc,   DN);
	resize_fill(m,       DN, .01);
	resize_zero(xSolver, DN); 
}

void SimulationMesh::prep_tendon_properties_dVectors() {
	resize_zero(balphaz, 0); 
	resize_zero(balphac, 0); 
}
 
////////////////////////////////////////////////////////////////////////////////
// solvers /////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

pair<dVector, dVector> SimulationMesh::solve_statics(const dVector &x_0, const dVector &balphac) { 
	dVector x_push       = this->x;
	dVector balphac_push = this->balphac;
	// --
	dVector x_new;
	dVector v_ZERO; resize_zero(v_ZERO, this->v.size());
	{

		energyFunction->setToStaticsMode(0.01);

		// dVector x_0 = X;
		this->x = x_0;
		this->update_contacts(x_0); // FORNOW
		this->balphac = balphac;
		this->xSolver = x_0;

		if (checkDerivatives) {
			energyFunction->testGradientWithFD(xSolver);
			energyFunction->testHessianWithFD(xSolver);
		}
		else if (checkGradients) {
			energyFunction->testGradientWithFD(xSolver);
		}
		else if (checkHessians) {
			energyFunction->testHessianWithFD(xSolver);
		}

		int    MAX_ITERATIONS = 10;
		double SOLVE_RESIDUAL = 10e-5;
		// if (HIGH_PRECISION_NEWTON) { MAX_ITERATIONS = 100; SOLVE_RESIDUAL = 10e-50; }
		if (HIGH_PRECISION_NEWTON) { MAX_ITERATIONS = 1000; SOLVE_RESIDUAL = 10e-50; }
		NewtonFunctionMinimizer minimizer(MAX_ITERATIONS);
		minimizer.solveResidual = SOLVE_RESIDUAL;

		minimizer.printOutput = true;
		double functionValue = energyFunction->computeValue(xSolver);
		minimizer.minimize(energyFunction, xSolver, functionValue);

		x_new = xSolver;

		if (is_nan(x_new)) { error("x_new is NaN"); } 
	}
	this->x       = x_push;
	this->balphac = balphac_push;

	return pair<dVector, dVector>(x_new, v_ZERO);
}

pair<dVector, dVector> SimulationMesh::solve_dynamics(const dVector &x_0, const dVector &v_0, const dVector &balphac) { 
	dVector x_push       = this->x;
	dVector v_push       = this->v;
	dVector balphac_push = this->balphac;
	// --
	dVector x_new;
	dVector v_new;
	{

		energyFunction->setToDynamicsMode(timeStep);

		this->update_contacts(x_0);
		this->v = v_0;
		this->balphac = balphac;
		this->x = x_0;
		this->xSolver = x_0;

		if (checkDerivatives) {
			energyFunction->testGradientWithFD(xSolver);
			energyFunction->testHessianWithFD(xSolver);
		}
		else if (checkGradients) {
			energyFunction->testGradientWithFD(xSolver);
		}
		else if (checkHessians) {
			energyFunction->testHessianWithFD(xSolver);
		}

		int    MAX_ITERATIONS = 10;
		double SOLVE_RESIDUAL = 10e-5;
		int MAX_LINE_SEARCH_ITERATIONS = 15;
		double LINE_SEARCH_START_VALUE = 1.;
		if (HIGH_PRECISION_NEWTON) {
			MAX_ITERATIONS = 1000;
			SOLVE_RESIDUAL = 1e-8;
			MAX_LINE_SEARCH_ITERATIONS = 48;
			LINE_SEARCH_START_VALUE = 1.;
		}
		NewtonFunctionMinimizer minimizer(MAX_ITERATIONS);
		minimizer.solveResidual = SOLVE_RESIDUAL;
		minimizer.maxLineSearchIterations = MAX_LINE_SEARCH_ITERATIONS;
		minimizer.lineSearchStartValue = LINE_SEARCH_START_VALUE;
		minimizer.printOutput = true;

		double functionValue = energyFunction->computeValue(xSolver); // TODO:See AppSoftIK notes.
		minimizer.minimize(energyFunction, xSolver, functionValue);

		x_new = xSolver;
		v_new = (x_new - x_0) / timeStep;

		if (is_nan(x_new)) {
			error("x_new is NaN");
			cout << " x_0 is_nan? " << is_nan(x_0)     << endl;
			cout << " v_0 is_nan? " << is_nan(v_0)     << endl;
			cout << "   u is_nan? " << is_nan(balphac) << endl;
			cout << "-------------" << endl;
			cout << " u: " << balphac.transpose() << endl;
			// __debugbreak(); 
		}

	}
	this->x       = x_push;
	this->v       = v_push;
	this->balphac = balphac_push;

	return pair<dVector, dVector>(x_new, v_new);
}

pair<dVector, dVector> SimulationMesh::solve_statics() {
	return solve_statics(this->x, this->balphac); 
}

pair<dVector, dVector> SimulationMesh::solve_dynamics() {
	return solve_dynamics(this->x, this->v, this->balphac); 
}

/*
void SimulationMesh::solve_contact_dynamics(double dt) {

	dVector x_old = x;
	dVector v_old = v;
	dVector f_ctc_old = f_ctc;

	contactSimEnergyFunction->timeStep = dt;

	int X_SIZE = contactSimEnergyFunction->X_SIZE();
	int fNfT_SIZE = D()*contactSimEnergyFunction->fN_SIZE();

	dVector optVecSolver;
	resize_zero(optVecSolver, X_SIZE + fNfT_SIZE);
	optVecSolver.head(X_SIZE) = x_old;
	optVecSolver.segment(X_SIZE, fNfT_SIZE) = contactSimEnergyFunction->reverse_augment(f_ctc_old);

	if (checkDerivatives) {
		contactSimEnergyFunction->testGradientWithFD(optVecSolver);
		contactSimEnergyFunction->testHessianWithFD(optVecSolver);
	} else if (checkGradients) {
		contactSimEnergyFunction->testGradientWithFD(optVecSolver);
	} else if (checkHessians) {
		contactSimEnergyFunction->testHessianWithFD(optVecSolver);
	}
 
	NewtonFunctionMinimizer minimizer(3);
	minimizer.solveResidual = 10e-50;
	minimizer.printOutput = true;

	{
		double functionValue = contactSimEnergyFunction->computeValue(optVecSolver);
		// cout << "Energy value before contact dynamics solve: " << functionValue << endl;
		minimizer.minimize(contactSimEnergyFunction, optVecSolver, functionValue);
	}

	dVector x_new = contactSimEnergyFunction->get_x(optVecSolver);
	dVector f_ctc_new = contactSimEnergyFunction->augment_f_ctc_with_zeroes_to_match_x(optVecSolver);

	// --

	if (is_nan(x_new)) { error("x_new is NaN"); }
	if (is_nan(f_ctc_new)) { error("f_ctc_new is NaN"); }

	x = x_new;
	v = (x_new - x_old) / dt;
	f_ctc = f_ctc_new;

	// NOTE: EXPERIMENTAL
	if (false) {
		x = x_old;
		v = v_old;
		f_ctc = f_ctc_new;
		solve_dynamics(dt);
	}
 
	// {
	// 	double functionValue = contactSimEnergyFunction->computeValue(optVecSolver);
	// 	cout << "Energy value after contact dynamics solve: " << functionValue << endl;
	// }
}
*/

////////////////////////////////////////////////////////////////////////////////
// mesh modification ///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void SimulationMesh::addGravityForces(const V3D& g) {
	for (uint i=0;i<nodes.size();i++)
		for (int j=0;j<nodes[i]->dimSize;j++)
			f_ext[nodes[i]->dataStartIndex + j] = g[j] * m[nodes[i]->dataStartIndex + j];
}

void SimulationMesh::fakeContactWithPlane(const Plane& plane) {
	for (uint i=0;i<nodes.size();i++){
		double dist = plane.getSignedDistanceToPoint(nodes[i]->getCurrentPosition());
		if (dist < 0){
			P3D newPos = nodes[i]->getCurrentPosition() + plane.n * (-dist);
			nodes[i]->setCurrentPosition(newPos);
			V3D vel = nodes[i]->getVelocity();
			if (vel.dot(plane.n) < 0){
				vel.setComponentAlong(plane.n, 0);
				nodes[i]->setVelocity(vel);
			}
		}
	}
}

 void SimulationMesh::removePinnedNodeConstraints() {
	// FORNOW
	for (auto it = pins.begin(); it != pins.end(); ++it)
		delete *it;
	pins.clear();
}

void SimulationMesh::clear() {
	nodes.clear();
	elements.clear();
	// --
	simplices.clear();
	tendons.clear();
	pins.clear();
	// --
	delete energyFunction;
	energyFunction = NULL;
}
 
////////////////////////////////////////////////////////////////////////////////
// addition ////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void SimulationMesh::add_node(Node *node) {
	nodes.push_back(node);
}

// FORNOW: Keep both around.
void SimulationMesh::add_nodes(const vector<P3D> &nodes_as_vecP3D) {
	add_N_nodes(nodes_as_vecP3D.size());
} 
//
void SimulationMesh::add_N_nodes(int N) {
	// NOTE: Adding to (N() != 0)-mesh _not_ tested.
	int offset = nodes.size();
	for (int i = offset; i < offset + N; i++) {
		Node* node = new Node(this, i, D()*i, D());
		add_node(node);
	}
}
 void SimulationMesh::init_nodal_positions(const vector<P3D> &nodes_as_vecP3D) {
	if (N() != nodes_as_vecP3D.size()) { error("Mesh mis-spec'd!"); }
	// --
	for (int n = 0; n < N(); ++n) {
		for (int d = 0; d < D(); ++d) {
			X[n*D() + d] = nodes_as_vecP3D[n][d];
		}
	}
	x       = X; 
	x_prime = X; 
	xSolver = X;
}
 
void SimulationMesh::add_simplex(SimMeshElement *simplex) {
	elements.push_back(simplex);
	simplices.push_back(simplex);
}

void SimulationMesh::add_tendon(Tendon * tendon) {
	tendon->tendonIndex = tendons.size(); // NOTE: We're appending 
	// --
	elements.push_back(tendon);
	tendons.push_back(tendon); 
	// --
	dVector balphaz_tmp = balphaz;
	dVector balphac_tmp = balphac;
	resize_zero(balphaz, tendons.size());
	resize_zero(balphac, tendons.size());
	for (Eigen::DenseIndex i = 0; i < balphaz_tmp.size(); ++i) {
		balphaz[i] = balphaz_tmp[i];
		balphac[i] = balphac_tmp[i];
	}

	if (tendon->tendonIndex != balphaz.size() - 1) { error("tendonIndex / balpha* mismatch"); }
	if (balphaz.size() != balphac.size()) { error("balphaz / balphac mismatch"); }

	balphaz[tendon->tendonIndex] = tendon->get_L(this->X); // 2.*  .9* ??? Will conflict with hack for no targets
	balphac[tendon->tendonIndex] = 0.;

	cout << "Added tendon " << tendon->tendonIndex << "." << endl;
}

void SimulationMesh::add_tendon_from_vecPoint(vector<Point *> tmp) {
	Tendon *tendon = new Tendon(this, tmp);
	add_tendon(tendon);
} 

void SimulationMesh::add_tendon_from_vecInt(vector<int> tmp) {
	vector<Point *> tmp2;
	for (auto &i : tmp) {
		tmp2.push_back(new Point(nodes[i]));
	}
	add_tendon_from_vecPoint(tmp2);
} 

// TODO: Picking for tendons (TODO: Mouse handler will have to have bool CHECK_NODES, CHECK_TENDONS, CHECK_BOTH)
// TODO: (Pass a list of tendons to the handler)
void SimulationMesh::delete_tendon(Tendon *tendon) {

	int tendon_i  = -1;
	int element_i = -1;

	// // -- Delete from lists.
	// Harvest indices.
	for (size_t chk_i = 0; chk_i < tendons.size(); ++chk_i) {
		auto &chk = tendons[chk_i];
		if (tendon == chk) { // FORNOW
			tendon_i = chk_i;
			break;
		}
	}
	// --
	for (size_t chk_i = 0; chk_i < elements.size(); ++chk_i) {
		SimMeshElement *element = elements[chk_i];
		Tendon *chk = dynamic_cast<Tendon *>(element);
		if (chk == NULL) {
			continue;
		}
		// --
		if (tendon == chk) { // FORNOW
			element_i = chk_i;
			break;
		}
	}
	if (tendon_i == -1 ) { error("Tendon not found in SimulationMesh::tendons." ); }
	if (element_i == -1) { error("Tendon not found in SimulationMesh::elements."); }

	// Destroy.
	remove_at(tendons,  tendon_i );
	remove_at(elements, element_i);
	// TODO: delete tendon;

	// // -- Update dVectors.
	dVector balphaz_tmp = balphaz;
	dVector balphac_tmp = balphac;
	resize_zero(balphaz, tendons.size());
	resize_zero(balphac, tendons.size());
	int j = -1;
	for (int i = 0; i < balphaz_tmp.size(); ++i) {
		if (i == tendon_i) {
			continue;
		}
		// --
		++j;
		balphaz[j] = balphaz_tmp[i];
		balphac[j] = balphac_tmp[i];
	}
}

void SimulationMesh::add_contact(Contact *contact) {
	contact->contactIndex = contacts.size(); // NOTE: We're appending 
	// --
	elements.push_back(contact);
	contacts.push_back(contact); 
}

void SimulationMesh::update_contacts(const dVector &x) {
	for (auto &contact : this->contacts) {
		contact->update(x);
	}
}

void SimulationMesh::analyze_lower_simplices() {

	auto sort_vecInt = [](vector<int> &v) {
		std::sort(v.begin(), v.end());
	};

	auto all_lower_simplices_as_vecVecInt = [this](const vector<int> &simplex_as_vecInt) {
		vector<vector<int>> ret;
		// FORNOW: Just casing out rather than using a recursive approach.
		if (D() == 2) {
			for (int i = 0; i < D() + 1; ++i) {
				for (int j = i + 1; j < D() + 1; ++j) {
					vector<int> tmp;
					tmp.push_back(simplex_as_vecInt[i]);
					tmp.push_back(simplex_as_vecInt[j]);
					ret.push_back(tmp);
				}
			}
		}
		else if (D() == 3) {
			for (int i = 0; i < D() + 1; ++i) {
				for (int j = i + 1; j < D() + 1; ++j) {
					for (int k = j + 1; k < D() + 1; ++k) {
						vector<int> tmp;
						tmp.push_back(simplex_as_vecInt[i]);
						tmp.push_back(simplex_as_vecInt[j]);
						tmp.push_back(simplex_as_vecInt[k]);
						ret.push_back(tmp);
					}
				}
			}
		}
		else {
			error("wat.");
		}
		return ret;
	};

	// cout << "(3) starting..." << endl; 
	std::map<vector<int>, vector<int>> low2high;
	std::map<vector<int>, int> times_found_map;
	for (auto &simplex_as_vecInt : simplices_as_vecVecInt) {
		for (auto &lower_simplex_as_vecInt : all_lower_simplices_as_vecVecInt(simplex_as_vecInt)) {
			sort_vecInt(lower_simplex_as_vecInt);
			// --
			low2high[lower_simplex_as_vecInt] = simplex_as_vecInt;
			times_found_map[lower_simplex_as_vecInt]++;
		}
	}
	// --
	for (const auto &pair : times_found_map) {
		vector<int> lower_simplex_as_vecInt = pair.first;
		int count = pair.second;
		// // construct
		// --
		vector<Node *> nodes_LS;
		for (auto &i : lower_simplex_as_vecInt) {
			nodes_LS.push_back(nodes[i]);
		}
		// --
		SimMeshElement *higher_simplex_LS;
		vector<int> high = low2high[lower_simplex_as_vecInt];
		for (size_t i = 0; i < simplices_as_vecVecInt.size(); ++i) {
			vector<int> check_high = simplices_as_vecVecInt[i];
			if (high == check_high) {
				higher_simplex_LS = simplices[i];
				break;
			}
		}
		// --
		LowerSimplex *lower_simplex = new LowerSimplex(nodes_LS, higher_simplex_LS);
		// --
		// cout << ". ";
		lower_simplices.push_back(lower_simplex);
		if (count == 1) {
			// cout << "* ";
			boundary_simplices.push_back(lower_simplex);
		}
		else if (count > 2) {
			error("Analysis of lower simplices found a broken mesh.");
		}
	}

	// NOTE: Orienting boundary simplices for 2D case
	if (D() == 2) {


		// NOTE Hackety hack.
		double min_y = INFINITY;
		for (auto &node : nodes) {
			double node_y = node->getUndeformedPosition().y();
			min_y = min(min_y, node_y);
		}
		// --
		int seed_i = -1;
		double max_x = -INFINITY;
		for (size_t i = 0; i < boundary_simplices.size(); ++i) {
			auto &edge = boundary_simplices[i];
			auto &node0 = edge->nodes[0];
			auto &node1 = edge->nodes[1];
			double node0_y = node0->getUndeformedPosition().y();
			double node1_y = node1->getUndeformedPosition().y();

			if (IS_EQUAL(node0_y, node1_y)) {
				continue;
			}

			auto &node = (node0_y < node1_y) ? node0 : node1;
			double node_x = node->getUndeformedPosition().x();
			double node_y = node->getUndeformedPosition().y();
			if (IS_EQUAL(node_y, min_y)) {
				if (node_x > max_x) { 
					seed_i = i;
					max_x = node_x;
				}
			} 
		}


		auto seed = remove_at(boundary_simplices, seed_i);
		// auto seed = random_pop(boundary_simplices);
		// NOTE: Initial check that we're headed counter clockwise.
		{
			auto &hs = seed->higher_simplex;
			vector<int> i_ls = { seed->nodes[0]->nodeIndex, seed->nodes[1]->nodeIndex };
			vector<int> &i_hs = hs->i_vec_;
			int i_other;
			for (auto &i : i_hs) {
				if (i != i_ls[0] && i != i_ls[1]) {
					i_other = i;
					break;
				}
			}
			auto &nodes = hs->simMesh->nodes; // NOTE: Uh...  Sure...
			V3D a = V3D(nodes[i_ls[0]]->getUndeformedPosition(), nodes[i_ls[1]]->getUndeformedPosition());
			V3D b = V3D(nodes[i_ls[0]]->getUndeformedPosition(), nodes[i_other]->getUndeformedPosition());
			if (a.cross(b).z() < 0) {
				std::reverse(seed->nodes.begin(), seed->nodes.end());
			}

		}
		// Now we just link 'em up.
		vector<LowerSimplex *> tmp = {seed};
		while (boundary_simplices.size() != 0) {
			int find_i = tmp.back()->nodes[1]->nodeIndex; 
			for (size_t i = 0; i < boundary_simplices.size(); ++i) {
				bool MATCHES_FIRST  = (boundary_simplices[i]->nodes[0]->nodeIndex == find_i);
				bool MATCHES_SECOND = (boundary_simplices[i]->nodes[1]->nodeIndex == find_i);
				if  (MATCHES_FIRST || MATCHES_SECOND) {
					auto ls = remove_at(boundary_simplices, i);
					if (MATCHES_SECOND) {
						std::reverse(ls->nodes.begin(), ls->nodes.end());
					}
					tmp.push_back(ls);
					break;
				}
			}
		}

		boundary_simplices = tmp;
	}

	for (auto &node : nodes) {
		bool ON_BOUNDARY = false;
		// NOTE: O(horrible)
		for (auto &bs : boundary_simplices) {
			for (auto *chk : bs->nodes) {
				if (node == chk) {
					ON_BOUNDARY = true;
				}
			} 
		}
		// --
		if (ON_BOUNDARY) {
			boundary_nodes.push_back(node);
		}
	}
}


void SimulationMesh::spawnMesh(const vector<P3D> &nodes_as_vecP3D, const vector<vector<int>> &simplices_as_vecVecInt) {
	// // // NOTE: All mesh construction should really run this function right here.
	// // (0) FORNOW: Intermediate storage. 
	this->nodes_as_vecP3D        = nodes_as_vecP3D;
	this->simplices_as_vecVecInt = simplices_as_vecVecInt;
	// // (1) Nodes.
	// (1a) Add nodes.
	add_nodes(nodes_as_vecP3D);
	cout << "(1a add_nodes) done." << endl;
	// (1b) Prep nodal vectors.
	prep_nodal_properties_dVectors();
	prep_nodal_force_dVectors();
	cout << "(1b prep) done." << endl;
	// // (2) Energy elements.
	// (2a) Init positions.
	init_nodal_positions(nodes_as_vecP3D);
	cout << "(2a init_nodal_positions) done." << endl;
	// (2b) Add simplices.
	add_simplices(simplices_as_vecVecInt);
	cout << "(2b add_simplices) done." << endl;
	// (2c) Initialize energy function(s).
	energyFunction  = new FEMEnergyFunction();
	energyFunction->initialize(this);
	// contactSimEnergyFunction  = new ContactSimEnergyFunction();
	// contactSimEnergyFunction->initialize(this);
	cout << "(2c energyFunction) done." << endl;
	// // (3) Lower simplices.
	analyze_lower_simplices();
	cout << "(3 analyze_lower_simplices) done." << endl;
	// TODO: Combine into method: setExternalForce(const V3D &);
	// NOTE: This method should remove any external forces, and also 
	// addGravityForces(V3D(-0., -10., -0.));
	// cout << "(4 addGravityForces) done." << endl;

}

void SimulationMesh::loadSavedTendons(const char *prefix) {
	char tdn_filename[128];
	apply_suffix(prefix, "tdn",     tdn_filename);
	FILE* tdn_fp = fopen(tdn_filename, "r");

	if (tdn_fp == nullptr) {
		cout << "(loadSavedTendons) Could not find tendon file." << endl;
		return;
	}

	vector<vector< pair<vector<int>, vector<double>> >> tendons_as_vecVecPairVecIntVecDouble;
	vector<int>    i_vec; i_vec.resize(D() + 1, -1);
	vector<double> L_vec; L_vec.resize(D() + 1, 0.);
	if (tdn_fp != nullptr) {
		// tendons_as_vecVecPairVecIntVecDouble
		const int LINESZ = 1024;
		char line[LINESZ]; 
		while (fgets(line, LINESZ, tdn_fp) != NULL) {

			vector<pair<vector<int>, vector<double>>> tendon;

			if (D() == 2) {
				// https://stackoverflow.com/questions/10826953/sscanf-doesnt-move-scans-same-integer-everytime-c 
				int nums_now, bytes_now;
				int bytes_consumed = 0, nums_read = 0;
				while ((nums_now = sscanf(line + bytes_consumed, "%d %lf %d %lf %d %lf %n", &i_vec[0], &L_vec[0], &i_vec[1], &L_vec[1], &i_vec[2], &L_vec[2], &bytes_now)) > 0) {
					bytes_consumed += bytes_now; nums_read += nums_now;
					tendon.push_back(make_pair(i_vec, L_vec));
				} 
			} else if (D() == 3) {
				// https://stackoverflow.com/questions/10826953/sscanf-doesnt-move-scans-same-integer-everytime-c 
				int nums_now, bytes_now;
				int bytes_consumed = 0, nums_read = 0;
				while ((nums_now = sscanf(line + bytes_consumed, "%d %lf %d %lf %d %lf %d %lf %n", &i_vec[0], &L_vec[0], &i_vec[1], &L_vec[1], &i_vec[2], &L_vec[2], &i_vec[3], &L_vec[3], &bytes_now)) > 0) {
					// cout << "i_vec :" << i_vec[0] << " " << i_vec[1] << " " << i_vec[2] << endl;
					// cout << "L_vec :" << L_vec[0] << " " << L_vec[1] << " " << L_vec[2] << endl;
					// cout << endl;
					bytes_consumed += bytes_now; nums_read += nums_now;
					tendon.push_back(make_pair(i_vec, L_vec));
				} 
			} else {
				error("IOError");
			}

			tendons_as_vecVecPairVecIntVecDouble.push_back(tendon);
		}
	}
	// --
	for (auto &tendon : tendons_as_vecVecPairVecIntVecDouble) {
		vector<Point *> tendon_as_vecPoint; 
		for (auto &point_spec : tendon) {
			vector<int>    i_vec_ = point_spec.first;
			vector<double> L_vec_ = point_spec.second; 
			vector<int>    i_vec;
			vector<double> L_vec;
			for (size_t j = 0; j < i_vec_.size(); ++j) {
				int    i = i_vec_[j];
				double L = L_vec_[j];
				if (i != -1 && L > 0 && !IS_ZERO(L)) {
					i_vec.push_back(i);
					L_vec.push_back(L);
				}
			}
			// --
			tendon_as_vecPoint.push_back(new Point(vecInt2vecNode(i_vec), L_vec));
		} 
		add_tendon_from_vecPoint(tendon_as_vecPoint);
	} 
	fclose(tdn_fp);
	cout << "(loadSavedTendons) Loaded tendons." << endl;
}

void SimulationMesh::pinMesh(const vector<int> &pins_as_vecInt) {
	for (int i : pins_as_vecInt) {
		setPinnedNode(nodes[i]->nodeIndex, nodes[i]->getUndeformedPosition()); // NOTE: ->nodeIndex _not_ tested
	} 
	cout << "(5 pinMesh) done." << endl;
}

void SimulationMesh::pinToFloor() {
	double min_y = HUGE;
	for (auto &node : nodes) {
		double y = node->getCurrentPosition()[1];
		min_y = min(y, min_y);
	}
	// --
	vector<int> pins_as_vecInt;
	for (auto &node : nodes) {
		int i = node->nodeIndex;
		P3D s = node->getCurrentPosition();
		if (abs(s[1] - min_y) < .01) {
			pins_as_vecInt.push_back(i);
		}
	}
	// --
	pinMesh(pins_as_vecInt);
}

void SimulationMesh::pinToCeiling() {
	double max_y = -HUGE;
	for (auto &node : nodes) {
		double y = node->getCurrentPosition()[1];
		max_y = max(y, max_y);
	}
	// --
	vector<int> pins_as_vecInt;
	for (auto &node : nodes) {
		int i = node->nodeIndex;
		P3D s = node->getCurrentPosition();
		if (abs(s[1] - max_y) < .01) {
			pins_as_vecInt.push_back(i);
		}
	}
	// --
	pinMesh(pins_as_vecInt);
}

void SimulationMesh::pinToLeftWall() {
	double min_x = HUGE;
	for (auto &node : nodes) {
		double x = node->getCurrentPosition()[0];
		min_x = min(x, min_x);
	}
	// --
	vector<int> pins_as_vecInt;
	for (auto &node : nodes) {
		int i = node->nodeIndex;
		P3D s = node->getCurrentPosition();
		if (abs(s[0] - min_x) < .001) {
			pins_as_vecInt.push_back(i);
		}
	}
	// --
	pinMesh(pins_as_vecInt);
}

////////////////////////////////////////////////////////////////////////////////
// test suite (TODO) //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void SimulationMesh::rig_01() {
	vector<Point *> tmp;
	tmp.push_back(new Point(nodes[0]));
	tmp.push_back(new Point(nodes[1]));
	add_tendon_from_vecPoint(tmp);
}

void SimulationMesh::rig_n0s1_bary() {
	vector<Point *> tmp;
	// --
	tmp.push_back(new Point(nodes[0]));
	// --
	vector<Node *> nodes_tmp;
	vector<double> weights_tmp;
	auto &simplex = simplices[0];
	for (int ii = 0; ii <= D(); ++ii) {
		// FORNOW
		nodes_tmp.push_back(nodes[simplex->i_vec_[ii]]);
		weights_tmp.push_back(1./((double) D() + 1.));
	}
	Point *wpt = new Point(nodes_tmp, weights_tmp);
	tmp.push_back(wpt);
	// --
	add_tendon_from_vecPoint(tmp);
}

void SimulationMesh::rig_random(int T) {
	vector<int> i_vec;
	for (int j = 0; j < N(); ++j) {
		i_vec.push_back(j);
	}
	// --
	vector<Point *> tmp;
	for (int _ = 0; _ < T; ++_) {
		tmp.push_back(new Point(nodes[random_pop(i_vec)]));
	}
	add_tendon_from_vecPoint(tmp);
} 

void SimulationMesh::rig_random_bary(int T) {
	vector<int> i_vec;
	for (int j = 0; j < N(); ++j) {
		i_vec.push_back(j);
	}
	// --
	vector<Point *> tmp;
	for (int _ = 0; _ < T; ++_) {
		auto &simplex = simplices[random_int(simplices.size())];
		// --
		dVector w;
		resize_zero(w, D() + 1);
		for (int ii = 0; ii <= D(); ++ii) { w[ii] = random_double(); }
		w /= w.sum();
		// --
		vector<Node *> nodes_tmp;
		vector<double> weights_tmp;
		for (int ii = 0; ii <= D(); ++ii) {
			nodes_tmp.push_back(nodes[simplex->i_vec_[ii]]);
			weights_tmp.push_back(w[ii]);
		}
		Point *wpt = new Point(nodes_tmp, weights_tmp);
		tmp.push_back(wpt);
	}
	add_tendon_from_vecPoint(tmp);
} 

void SimulationMesh::rig_boundary_simplices() {
	vector<vector<Point *>> pairs;
	// --
	for (auto &bs : boundary_simplices) {
		auto &n = bs->nodes;
		auto magic = [&](int a, int b) {pairs.push_back({ new Point(n[a]), new Point(n[b]) }); };
		if (n.size() == 2) {
			magic(0, 1);
		}  else if (n.size() == 3) {
			magic(0, 1);
			magic(1, 2);
			magic(2, 0);
		} else {
			error("bs appears broken.");
		} 
	} 
	// --
	for (auto &pair : pairs) {
		add_tendon_from_vecPoint(pair);
	}
}

void SimulationMesh::rig_all_lower_simplices() {
	vector<vector<Point *>> pairs;
	// --
	for (auto &ls : lower_simplices) {
		auto &n = ls->nodes;
		auto magic = [&](int a, int b) {pairs.push_back({ new Point(n[a]), new Point(n[b]) }); };
		if (n.size() == 2) {
			magic(0, 1);
		}  else if (n.size() == 3) {
			magic(0, 1);
			magic(1, 2);
			magic(2, 0);
		} else {
			error("ls appears broken.");
		} 
	} 
	// --
	for (auto &pair : pairs) {
		add_tendon_from_vecPoint(pair);
	}
}

void SimulationMesh::add_contacts_to_boundary_nodes() {
	for (auto &boundary_node : boundary_nodes) {
		add_contact(new Contact(boundary_node, this));
	}
}

void SimulationMesh::relax_tendons() { 
	for (auto &tendon : tendons) {
		tendon->set_alphac(-tendon->get_alphaz());
	}
}

void SimulationMesh::nudge_mesh(const V3D &t) {
	for (auto &node : nodes) { node->setUndeformedPosition(node->getUndeformedPosition() + t); }
	x = X; x_prime = X; xSolver = X;
}

void SimulationMesh::nudge_mesh_up() {
	for (auto &node : nodes) { node->setUndeformedPosition(node->getUndeformedPosition() + V3D(0., .5)); }
	x = X; x_prime = X; xSolver = X;
}

void SimulationMesh::rotate_mesh(const double &theta) {
	if (D() != 2) { error("rotate_mesh: [NotImplementedError]"); }
	for (auto &node : nodes) { node->setUndeformedPosition(rotate_2D(node->getUndeformedPosition(), theta)); }
	x = X; x_prime = X; xSolver = X;
}

void SimulationMesh::move_pins(const dVector &x) { 
	for (auto &pin : pins) {
		auto pin_ = dynamic_cast<FixedPointSpring2D *>(pin);
		pin_->targetPosition = pin_->node->getCoordinates(x);
	}
}

////////////////////////////////////////////////////////////////////////////////
// nodal forces ///////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void SimulationMesh::prep_nodal_force_dVectors() { 
	int DN = D()*N();
	// --
	resize_zero(F,      DN);
	resize_zero(F_spx,  DN);
	resize_zero(F_pin,  DN);
	resize_zero(F_tdn,  DN);
	resize_zero(F_ext,  DN);
	resize_zero(F_ctc,  DN);
	resize_zero(F_sum,  DN);
}

void SimulationMesh::computeTotalNodalForces(const dVector &y) {
	// F: Total nodal force|y (considers everything, including dynamics)
	//    NOTE: In statics, F = F_sum.
	F.setZero();
	energyFunction->addGradientTo(F, y);
	F *= -1;
}

void SimulationMesh::computeConstituentNodalForces(const dVector &y) {
	/* Computes total nodal forces, as well as useful constituent forces.
	   Usage note: y is typically going to be x or x_prime. */
 
	// F_spx: Simplex (material) forces
	F_spx.setZero();
	for (auto &simplex : simplices) { simplex->addEnergyGradientTo(y, X, F_spx); }
	F_spx *= -1;

	// F_tdn: Tendon forces
	F_tdn.setZero();
	for (auto &tendon : tendons) { tendon->addEnergyGradientTo(y, X, F_tdn); }
	F_tdn *= -1;
 
	// F_pin: Pin forces
	F_pin.setZero();
	for (auto &pin : pins) { pin->addEnergyGradientTo(y, X, F_pin); }
	F_pin *= -1;

	// F_ext: External forces (gravity?)
	F_ext = f_ext; // FORNOW

	// F_ctc: Contact forces
	// F_ctc = f_ctc; // TODO: CHECKME
	F_ctc.setZero();
	for (auto &ctc : contacts) { ctc->addEnergyGradientTo(y, X, F_ctc); }
	F_ctc *= -1;

	// F_sum: Summed force (_not_ counting intertial term used for dynamics)
	F_sum = F_spx + F_tdn + F_pin + F_ext + F_ctc;

}

void SimulationMesh::drawNodalForces() {
	// NOTE: Need to call this first.
	computeTotalNodalForces(x);
	computeConstituentNodalForces(x);

	auto quiver_caller = [this] (const dVector &y, const dVector &G, const P3D &color) {
		vector<P3D> y_vec;
		vector<V3D> G_vec;
		if (D() == 2) {
			y_vec = pack_dVector2_into_vecP3D(y);
			G_vec = pack_dVector2_into_vecV3D(G);
		} else {
			y_vec = pack_dVector3_into_vecP3D(y);
			G_vec = pack_dVector3_into_vecV3D(G);
		}
		set_color(color);
		// --
		quiver(y_vec, G_vec, D());
	};

	auto incTr = [this]() {
		if (this->D() == 2) {
			glTranslated(0., 0., .0001);
		}
	};

	glPushMatrix(); {
		if (DRAW_F_SPX) { quiver_caller(x, F_spx, COL_F_SPX); } incTr();
		if (DRAW_F_PIN) { quiver_caller(x, F_pin, COL_F_PIN); } incTr();
		if (DRAW_F_TDN) { quiver_caller(x, F_tdn, COL_F_TDN); } incTr();
		if (DRAW_F_EXT) { quiver_caller(x, F_ext, COL_F_EXT); } incTr();
		if (DRAW_F_CTC) { quiver_caller(x, F_ctc, COL_F_CTC); } incTr();
		if (DRAW_F_SUM) { quiver_caller(x, F_sum, COL_F_SUM); } incTr();
	} glPopMatrix();
}

void SimulationMesh::drawNodalVelocities() {

	// FORNOW
	auto quiver_caller = [this](const dVector &y, const dVector &G, const P3D &color) {
		vector<P3D> y_vec;
		vector<V3D> G_vec;
		if (D() == 2) {
			y_vec = pack_dVector2_into_vecP3D(y);
			G_vec = pack_dVector2_into_vecV3D(G);
		}
		else {
			y_vec = pack_dVector3_into_vecP3D(y);
			G_vec = pack_dVector3_into_vecV3D(G);
		}
		set_color(color);
		// --
		quiver(y_vec, G_vec, D());
	};

	glPushMatrix(); {
		quiver_caller(x, v, COL_V);
	} glPopMatrix();

}

/*

void SimulationMesh::solve_statics_BFGS() {

	energyFunction->setToStaticsMode(0.01);

	xSolver = x;

	if (checkDerivatives)      { energyFunction->testGradientWithFD(xSolver);
								 energyFunction->testHessianWithFD(xSolver);
	} else if (checkGradients) { energyFunction->testGradientWithFD(xSolver);
	} else if (checkHessians)  { energyFunction->testHessianWithFD(xSolver);
	}
 
	int    MAX_ITERATIONS;
	double SOLVE_RESIDUAL;
	if (HIGH_PRECISION_STATICS) { MAX_ITERATIONS = 1000; SOLVE_RESIDUAL = 10e-50; }
	else                        { MAX_ITERATIONS = 50;   SOLVE_RESIDUAL = 10e-5;  }
	BFGSFunctionMinimizer minimizer(MAX_ITERATIONS);
	minimizer.solveResidual = SOLVE_RESIDUAL;

	minimizer.printOutput = true;
	double functionValue = energyFunction->computeValue(xSolver);
	minimizer.minimize(energyFunction, xSolver, functionValue); 
	v.setZero();

	if (!is_nan(xSolver)) { x = xSolver; } else { error("statics NaN"); }
}

*/