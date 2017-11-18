#pragma once

#include <Utils/Utils.h>
#include <RBSimLib/RBState.h>
#include <RBSimLib/RBProperties.h>
#include <GUILib/GLMesh.h>
#include <RBSimLib/CollisionDetectionPrimitive.h>
#include <MathLib/Transformation.h>

class Force;
class ArticulatedFigure;

//drawing flags
#define SHOW_MESH						0x0001
#define SHOW_BODY_FRAME					0x0002
#define SHOW_CD_PRIMITIVES				0x0004
#define SHOW_MOI_BOX					0x0008
#define SHOW_JOINTS						0x0010
#define SHOW_MATERIALS					0x0020
#define SHOW_ABSTRACT_VIEW				0x0040
#define HIGHLIGHT_SELECTED				0x0080

struct MappingInfo {
	int index1 = -1;
	int index2 = -1;
};

/*==================================================================================================================================================*
 | Rigid Body Class. It is assumed that the location of the center of mass of the rigid body corresponds to local coordinates (0,0,0).              |
 *==================================================================================================================================================*/
class Joint;
class RigidBody  {
public:
	// state of the rigid body: rigid body's position in the world, its orientation and linear/angular velocities (all stored in world coordinates)
	RBState state;
	// physical properties
	RBProperties rbProperties;
	// collision detection primitives that are relevant for this rigid body
	DynamicArray<CollisionDetectionPrimitive*> cdps;
	// meshes that are used to visualize the rigid body
	DynamicArray<GLMesh*> meshes;
	// meshes that are used to do carving to make room for the real meshes.
	DynamicArray<GLMesh*> carveMeshes;
	// meshTransformation corresponding to meshes.
	DynamicArray<Transformation> meshTransformations;
	// discriptions of the each mesh
	DynamicArray<std::string> meshDescriptions;

	// name of the rigid body
	std::string name;
	// id of the rigid body
	int id = -1;

	int fbxBoneId = -1;

	//for ease of access, we will keep a list of the joints that list this rigid body as a parent (child joints)
	DynamicArray<Joint*> cJoints;
	//and a list of joints that list this rigid body as a child (parent joints)
	DynamicArray<Joint*> pJoints;

	MappingInfo mappingInfo;

	bool selected = false;

	bool inContact = false;

	double abstractViewCylinderRadius = 0.01;

public:
	/**
		Default constructor
	*/
	RigidBody(void);

	/**
		Default destructor
	*/
	virtual ~RigidBody(void);

	/**
		This method returns the coordinates of the point that is passed in as a parameter(expressed in local coordinates), in world coordinates.
	*/
	P3D getWorldCoordinates(const P3D& localPoint) const;

	/**
		This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
	*/
	P3D getLocalCoordinates(const P3D& globalPoint);

	/**
		This method is used to return the local coordinates of the vector that is passed in as a parameter (expressed in global coordinates)
	*/
	V3D getLocalCoordinates(const V3D& globalVector);

	/**
		This method returns the vector that is passed in as a parameter(expressed in local coordinates), in world coordinates.
	*/
	V3D getWorldCoordinates(const V3D& localVector) const;

	inline std::string getName(){
		return name;
	}

	/**
		This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
		resulting velocity will be expressed in world coordinates.
	*/
	V3D getAbsoluteVelocityForLocalPoint(const P3D& localPoint);

	/**
		This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in global coordinates, and the
		resulting velocity will be expressed in world coordinates.
	*/
	V3D getAbsoluteVelocityForGlobalPoint(const P3D& globalPoint);

	/**
		This method returns the world coordinates of the position of the center of mass of the object
	*/
	inline P3D getCMPosition() const {
		return state.position;
	}

	/**
		This method sets the world coordinate of the posision of the center of mass of the object
	*/
	inline void setCMPosition(const P3D& newCMPos){
		state.position = newCMPos;		
	}

	/**
		This method returns the body's center of mass velocity
	*/
	inline V3D getCMVelocity(){
		return state.velocity;
	}


	/**
		This method returns the rigid body's angular velocity
	*/
	inline V3D getAngularVelocity() {
		return state.angularVelocity;
	}

	/**
		This method returns the rigid body's orientation
	*/
	inline Quaternion getOrientation() {
		return state.orientation;
	}

	/**
		This method sets the velocity of the center of mass of the object
	*/
	inline void setCMVelocity(const V3D& newCMVel){
		state.velocity = newCMVel;
	}

	/**
		this method sets the angular velocity of the body
	*/
	inline void setAngularVelocity(const V3D& newAVel){
		state.angularVelocity = newAVel;
	}

	/**
		This method returns the rigid body's coefficient of restitution
	*/
	inline double getRestitutionCoefficient(){
		return rbProperties.restitutionCoeff;
	}

	/**
		This method returns the rigid body's coefficient of restitution
	*/
	inline double getFrictionCoefficient(){
		return rbProperties.frictionCoeff;
	}

	/**
		This method draws the rigid body frame of reference
	*/
	virtual void drawAxes();

	/**
		This method draws the current rigid body.
	*/
	virtual void draw(int flags, V3D color, double alpha);
	virtual void draw(int flags, double alpha = 1.0);

	/**
		This method renders the rigid body in its current state as a set of vertices 
		and faces that will be appended to the passed OBJ file.

		vertexIdxOffset indicates the index of the first vertex for this object, this makes it possible to render
		multiple different meshes to the same OBJ file
		 
		Returns the number of vertices written to the file
	*/
	uint renderToObjFile(FILE* fp, uint vertexIdxOffset);

	// returns the world coords moment of inertia of the rigid body if it was to rotate about p, which is also expressed in world coords
	Matrix3x3 getWorldMOIAboutPoint(const P3D& p);

	// returns the world coords moment of inertia of the rigid body
	Matrix3x3 getWorldMOI();

	// returns the world coords moment of inertia of the rigid body
	Matrix3x3 getWorldMOI(const Quaternion& orientation);

	/**
		This method loads all the pertinent information regarding the rigid body from a file.
	*/
	void loadFromFile(FILE* fp);

	/**
		writes the RB to file
	*/
	void writeToFile(FILE* fp);

	/**
		this method sets the id of the current rigid body.
	*/
	inline void setBodyID(int newID){
		this->id = newID;
	}

	/**
		this method is used to draw the box derived using MOI
	*/
	void drawMOIBox();

	/*returns true if it is hit, false otherwise. */
	bool getRayIntersectionPointTo(const Ray& ray, P3D* localCoordsIntersectionPoint);

	void autoGenerateCDPs();


};

