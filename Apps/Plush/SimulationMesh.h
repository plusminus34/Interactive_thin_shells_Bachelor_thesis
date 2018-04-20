#pragma once

#include "Node.h"
#include "SimMeshElement.h"
#include "FEMEnergyFunction.h"
#include "Tendon.h"
#include "Contact.h"
#include "LowerSimplex.h"
// --
#include <GUILib/GLTexture.h>

class SimulationMesh {

// basics
public:
	SimulationMesh();
	~SimulationMesh();
	void clear();

public:
	void applyYoungsModulusAndPoissonsRatio(const double &, const double &);

public:
	double get_totalMass();
	P3D get_COM(const dVector &y);
	MatrixNxM get_dCOMdx();

// FORNOW
public:
	bool UNILATERAL_TENDONS = true;
	double timeStep = .01;
	int _DYNAMICS_MAX_ITERATIONS = 200;
	double _DYNAMICS_SOLVE_RESIDUAL = 1e-10;

// core suite
public:
	// energy
	vector<Node *>            nodes;                // All nodes.
	vector<SimMeshElement *>  elements;             // All energy elements.
	vector<SimMeshElement *>  non_E_elements;       // Other elements (draw but don't count in E, g, H).
	FEMEnergyFunction *       energyFunction;
	// ContactSimEnergyFunction *contactSimEnergyFunction;
	// convenience vectors
	vector<SimMeshElement *> simplices;            // D()-dimensional CSTElement's.
	vector<Tendon *>         tendons;              // Tendon's
	vector<SimMeshElement *> pins;                 // D()-dimensional FixedPointSpring's.
	vector<Contact *>    contacts;
	// --
	bool DRAW_ALL_ELEMENTS_OVERRIDE = false;
	bool DRAW_SIMPLICES = true;
	bool DRAW_TENDONS   = true;
	bool DRAW_PINS      = true;
	bool DRAW_ANCHORS   = false;
	bool DRAW_NON_E     = true;
	bool SECONDARY = false; // FORNOW

	// boundary
	vector<LowerSimplex *>   lower_simplices;      // Boundary work.
	vector<LowerSimplex *>   boundary_simplices;   // Boundary work.
	vector<Node *>           boundary_nodes;       // Boundary work.
	// vector<vector<Node *>>   boundary_nodes;       // Boundary work.
	// bool                     BOUNDARY_IDENTIFIED;  // Boundary work.
	// convenience functions
	virtual int D() = 0;  // Dimensionality of mesh. 
	int         N();      // nodes.size()
	int         C();      // contacts.size()

// FORNOW: intermediate storage formats
public:
	vector<P3D> nodes_as_vecP3D;
	vector<vector<int>> simplices_as_vecVecInt;

// nodal properties storage
public:
	dVector X, x, x_prime; // rest, def'd, and target pose
	dVector v, m, f_ext;   // velocity, mass, external forces
	dVector f_ctc;
	dVector xSolver;       // dummy vector for solver
	void prep_nodal_properties_dVectors();
	// --
	void propogate_X() {
		this->x       = this->X;
		this->x_prime = this->X;
		this->xSolver = this->X; 
	}
 
// vector properties storage
	dVector balphaz; // Assembly length.
	dVector balphac; // Contraction.
	void prep_tendon_properties_dVectors();
 
// addition / removal suite (respects convenience vectors)
//     TODO: removal
public:
	void add_node(Node *);
	void add_simplex(SimMeshElement *);
	void add_tendon(Tendon *);
	void add_tendon_from_vecPoint(vector<Point *>);
	void add_tendon_from_vecInt(vector<int>);
	void add_contact(Contact *);
	void update_contacts(const dVector &);
	void delete_tendon(Tendon *);

// mesh construction
public:
	void add_N_nodes(int);
	void add_nodes(const vector<P3D> &);
	void init_nodal_positions(const vector<P3D> &);
	virtual void add_simplices(const vector<vector<int>> &) = 0;
	void analyze_lower_simplices();
	void pinMesh(const vector<int> &);
	void pinToFloor();
	void pinToCeiling();
	void pinToLeftWall();
	void spawnMesh(const vector<P3D> &, const vector<vector<int>> &);
	virtual void spawnSavedMesh(const char *prefix, bool loadTendons=false) = 0;
	void loadSavedTendons(const char *prefix);

// simple utility/testing suite
public:
	virtual void spawnSimplexMesh() = 0;
	void rig_01();
	void rig_n0s1_bary();
	void rig_random(int);
	void rig_random_bary(int);
	void rig_boundary_simplices();
	void rig_all_lower_simplices();
	void add_contacts_to_boundary_nodes();
	void relax_tendons();
	void nudge_mesh(const V3D &t);
	void rotate_mesh(const double &);
	void nudge_mesh_up();
	void move_pins(const dVector &);

// tendon suite I (TODO: what was this gonna be again?)
public:
	;
// --
// tendon suite II (common params)
double c = 1e5;
double eps = .01;


// TODO: anchor point suit II
 
// nodal forces I (storage)
public: 
	dVector F, F_spx, F_pin, F_tdn, F_ctc, F_sum, F_ext;
	// --
	// void computeTotalNodalForces(const dVector &y);
	void computeConstituentNodalForces(const dVector &x, const dVector &alphac, const dVector &x0);
	void prep_nodal_force_dVectors();
// --
// nodal forces II (drawing)
public:
	bool DRAW_NODAL_FORCES = false;
	bool DRAW_F_SPX = true; const P3D COL_F_SPX = HENN1NK;
	bool DRAW_F_PIN = true; const P3D COL_F_PIN = PUMPKIN;
	bool DRAW_F_TDN = true; const P3D COL_F_TDN = RATIONALITY;
	bool DRAW_F_EXT = true; const P3D COL_F_EXT = LAVISH;
	bool DRAW_F_CTC = true; const P3D COL_F_CTC = ORCHID;
	bool DRAW_F_SUM = true; const P3D COL_F_SUM = GOLDCLOVER;
	// --
	void drawNodalForces(const dVector &x, const dVector &alphac, const dVector &x0);

public:
	//bool DRAW_NODAL_VELOCITIES = false; const P3D COL_V = WHITE;
	// --
	//void drawNodalVelocities(const dVector &x);

public:
	bool DRAW_BOUNDARY = true;
	bool DRAW_BOUNDARY_TRANSPARENT = false;

// solving
public: 
	bool HIGH_PRECISION_NEWTON = false;
	pair<dVector, dVector> solve_statics(const dVector &x_0, const dVector &balphac);
	pair<dVector, dVector> solve_dynamics(const dVector &x_0, const dVector &v_0, const dVector &balphac);
	// --
	pair<dVector, dVector> solve_statics();
	pair<dVector, dVector> solve_dynamics();
	// --
	void xvPair_INTO_Mesh(const pair<dVector, dVector> &xv) {
		this->x = xv.first;
		this->v = xv.second;
	}
	// --
	bool checkDerivatives, checkGradients, checkHessians;

// conversions
public: 
	vector<Node *> vecInt2vecNode(const vector<int> &i) {
		vector<Node *> vecNodes;
		for (auto &nodeIndex : i) {
			vecNodes.push_back(this->nodes[nodeIndex]);
		}
		return vecNodes;
	};
 
// stelian pure virtuals
public:
	virtual int  getSelectedNodeID(Ray ray) = 0;
	virtual void setPinnedNode(int ID, const P3D& target) = 0; 

// stelian impure virtuals
public:
	virtual void fakeContactWithPlane(const Plane &plane);
	virtual void draw(dVector &x=dVector(), dVector &alphac=dVector(), dVector &x0=dVector()); // TODO: Switch to taking v, and getting x0 from difference
	virtual void addGravityForces(const V3D &g);
	virtual void removePinnedNodeConstraints();

public:
	double magG_tmp;
	

};
