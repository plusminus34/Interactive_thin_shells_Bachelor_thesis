#pragma once

#define LAYER_THICKNESS 0.16

/**
====================================================
 	Computational Design of Mechanical Assemblies
====================================================
*/	
#include <vector>
#include <MathLib/Quaternion.h>
#include <GUILib/GLMesh.h>
#include <GUILib/OBJReader.h>
#include <MathLib/Matrix.h>
#include <MathLib/Ray.h>
#include <MathLib/BoundingBox.h>

class KS_MechanicalComponent;

class TracerParticle{
public:
	KS_MechanicalComponent * mc;
	P3D pLocal;

	DynamicArray<P3D> trajectory;

	TracerParticle(KS_MechanicalComponent *mc_, P3D pLocal_){
		this->mc = mc_;
		this->pLocal = pLocal_;
	}

	TracerParticle() : mc(NULL), pLocal(P3D(0, 0, 0)) {
	}

	void addTrajectoryPoint();
};

class KS_MechanicalComponent{
	friend class KineSimApp;
public:
	KS_MechanicalComponent(const char* name);
	virtual ~KS_MechanicalComponent(void);
	
	void setName(const char* name) { strcpy(this->m_name, name); }
	char* getName() {return m_name;}
	virtual void setupGeometry() = 0;
	void clearTracerParticles();
	void addTracerParticlesToList(DynamicArray<P3D>& tracerParticleList);

	double getAlpha() const { return alpha; }
	double getBeta() const { return beta; }
	double getGamma() const { return gamma; }
	V3D getAlphaAxis(){return n_alpha;}
	V3D getBetaAxis(){return n_beta;}
	V3D getGammaAxis(){return n_gamma;}
	void setAlphaAxis(const V3D& alphaAxis) {n_alpha = alphaAxis; n_beta.normalize(); setAngles(gamma, beta, alpha);}
	void setBetaAxis(const V3D& betaAxis){n_beta = betaAxis; n_beta.normalize(); setAngles(gamma, beta, alpha);}
	void setGammaAxis(const V3D& gammaAxis){n_gamma = gammaAxis; n_gamma.normalize(); setAngles(gamma, beta, alpha);}
	
	void setAngles(double val_gamma, double val_beta, double val_alpha);

	void setWorldCenterPosition(const P3D& pos);
	P3D getWorldCenterPosition() const;

	DynamicArray<P3D> getPoints_list() { return points_list; }
	void setPoints_list(DynamicArray<P3D>& Points_list) { points_list= Points_list; }

	virtual double getThickness(){return 0;}

	void setComponentIndex(int i) {cIndex = i;}
	int getComponentIndex() {return cIndex;}
	
	//w represents the world coordinates for the point (or vector) x, which is expressed in the local coordinate frame of the component. 
	P3D get_x(const P3D& w) const;
	V3D get_x(const V3D& w) const;
	//w represents the world coordinates for the point (or vector) x, which is expressed in the local coordinate frame of the component. The function works both for local points or vectors
	P3D get_w(const P3D& x) const;
	V3D get_w(const V3D& x) const;
	//return the jacobian that relates the change in world coordinates w with the change in state s=(gamma, beta, alpha, p.x, p.y, p.z)
	void get_dw_ds(const P3D& x, MatrixNxM& dw_ds);
	void get_dw_ds(const V3D& x, MatrixNxM& dw_ds);
	//return the matrix that relates the change in change of w with the change in rotation angle alpha
	void get_ddw_dads(const P3D& x, MatrixNxM& ddw_das);
	void get_ddw_dads(const V3D& x, MatrixNxM& ddw_das);
	//return the matrix that relates the change in change of w with the change in rotation angle beta
	void get_ddw_dbds(const P3D& x, MatrixNxM& ddw_dbs);
	void get_ddw_dbds(const V3D& x, MatrixNxM& ddw_dbs);
	//return the matrix that relates the change in change of w with the change in rotation angle gamma
	void get_ddw_dgds(const P3D& x, MatrixNxM& ddw_dgs);
	void get_ddw_dgds(const V3D& x, MatrixNxM& ddw_dgs);

	virtual bool loadFromFile(FILE* f) = 0;
	virtual bool writeToFile(FILE* f) = 0;
	virtual KS_MechanicalComponent* clone() const = 0;

	static int getStateSize(){ return m_stateSize;}

	int getNumOfMeshes(){return (int)meshes.size();}
	GLMesh* getMesh(uint i){return meshes[i];}
	void addMesh(GLMesh* m){meshes.push_back(m);}
	void removeMesh(uint i){delete meshes[i]; meshes.erase(meshes.begin()+i);}
	void clearMeshes(){meshes.clear();}
	

	void updateTracerParticles();
	void draw();
	void drawTracerParticles();

	int getTracerParticleCount(){
		return (int)tracerParticles.size();
	}

	P3D getTracerParticlePosition(int i);
	P3D getTracerParticleLocalPosition(int i);

	TracerParticle& getTracerParticle(int i){
		return tracerParticles[i];
	}

	void setTracerParticles(DynamicArray<TracerParticle> v) { tracerParticles.clear(); tracerParticles.resize(v.size()); std::copy(v.begin(), v.end(), tracerParticles.begin());}

	void addCylinderMesh(int nrVerts, double radius, double length, P3D localCoords, V3D v, bool setMeshColor = false);

	/**
		This method renders the mechanical component in its current state as a set of vertices 
		and faces that will be appended to the passed OBJ file.

		vertexIdxOffset indicates the index of the first vertex for this object, this makes it possible to render
		multiple different meshes to the same OBJ file
		 
		Returns the number of vertices written to the file
	*/
	uint renderToObjFile(FILE* fp, uint vertexIdxOffset, double scale = 1.0);

protected:
	//this is the collection of meshes for this component
	DynamicArray<GLMesh*> meshes;
	//name used to ID components when loading/writing to file
	char m_name[100];
	//the index of the component in the mechanical assembly
	int cIndex;
	//the index of the component in the mechanical assembly
	static int m_stateSize;

	//for planar mechanisms, this layer number is a quick way of offsetting the components along the z-axis...
	int layerNumber;

	//visualization only...
	bool selected;

	//State of the component is defined by its position and rotation. The rotation is defined by Euler angles. To avoid gimbal locks, the rotation
	//axes of the last two euler angles can be changed as needed. 
	P3D position;							//position of component in world coordinates
	double alpha, beta, gamma;					//rotation angles along the normal of the component
	V3D n_alpha, n_beta, n_gamma;			//rotation axes (n_alpha is special because it represents the component's local coordinates normal)
	//from the quantities above we compute these quaternions for ease of use
	Quaternion R_alpha, R_beta, R_gamma;

	DynamicArray<P3D> points_list;// add points here for creating mesh from the convexhull

	DynamicArray<TracerParticle> tracerParticles;
	//returns true if the input line was processed, false otherwise

	bool processInputLine(char* line);

	void loadTriangleMeshFromFile(char* fName);

	void writeBaseComponentToFile(FILE* f);

};

