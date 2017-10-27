#include <GUILib/GLUtils.h>
#include <RBSimLib/RigidBody.h>
#include <GUILib/OBJReader.h>
#include <RBSimLib/RBUtils.h>
#include <GUILib/GLContentManager.h>
#include <RBSimLib/Joint.h>

#include <Utils/Utils.h>
#include <GUILib/GLShaderMaterial.h>


/**
	Default constructor - give sensible values to the class members
*/
RigidBody::RigidBody(void){
}

/**
	Default destructor - free up all the memory that we've used up
*/
RigidBody::~RigidBody(void){
	//note: the meshes will be handled by the GLContentManager class

	for (uint i=0;i<cdps.size();i++)
		delete cdps[i];
}

/**
	This method returns the coordinates of the point that is passed in as a parameter(expressed in local coordinates), in world coordinates.
*/
P3D RigidBody::getWorldCoordinates(const P3D& localPoint){
	return this->state.getWorldCoordinates(localPoint);
}

/**
	This method returns the vector that is passed in as a parameter(expressed in local coordinates), in world coordinates.
*/
V3D RigidBody::getWorldCoordinates(const V3D& localVector){
	return this->state.getWorldCoordinates(localVector);
}

/**
	This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
*/
P3D RigidBody::getLocalCoordinates(const P3D& globalPoint){
	return this->state.getLocalCoordinates(globalPoint);
}

/**
	This method is used to return the local coordinates of the vector that is passed in as a parameter (expressed in global coordinates)
*/
V3D RigidBody::getLocalCoordinates(const V3D& globalVector){
	return this->state.getLocalCoordinates(globalVector);
}

/**
	This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
	resulting velocity will be expressed in world coordinates.
*/
V3D RigidBody::getAbsoluteVelocityForLocalPoint(const P3D& localPoint){
	return this->state.getAbsoluteVelocityForLocalPoint(localPoint);
}

/**
	This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in global coordinates, and the
	resulting velocity will be expressed in world coordinates.
*/
V3D RigidBody::getAbsoluteVelocityForGlobalPoint(const P3D& globalPoint){
	return this->state.getAbsoluteVelocityForGlobalPoint(globalPoint);
}

/**
This method draws the rigid body frame of reference
*/
void RigidBody::drawAxes()
{
	P3D x0(state.position[0], state.position[1], state.position[2]);
	
	glColor3d(1, 0, 0);
	drawArrow(x0, x0 + state.orientation.rotate(V3D(1, 0, 0))*0.1, 0.01);
	glColor3d(0, 1, 0);
	drawArrow(x0, x0 + state.orientation.rotate(V3D(0, 1, 0))*0.1, 0.01);
	glColor3d(0, 0, 1);
	drawArrow(x0, x0 + state.orientation.rotate(V3D(0, 0, 1))*0.1, 0.01);

}

/**
	This method draws the current rigid body.
*/
void RigidBody::draw(int flags, V3D color, double alpha) {
	//multiply the gl matrix with the transformations needed to go from local space into world space
	glPushMatrix();

	glPushAttrib(GL_LIST_BIT | GL_CURRENT_BIT | GL_ENABLE_BIT | GL_TRANSFORM_BIT);

	//set up the OpenGL transformation matrices to match the rotation and position of the rigid body - meshes, CDPs, etc, are stored in local coordinates	
	//translation part
	glTranslated(state.position[0], state.position[1], state.position[2]);

	//and rotation part
	V3D rotAxis; double rotAngle;
	state.orientation.getAxisAngle(rotAxis, rotAngle);
	glRotated(DEG(rotAngle), rotAxis[0], rotAxis[1], rotAxis[2]);

	glPushAttrib(GL_ENABLE_BIT);

	//draw the collision detection primitives if any
	if (flags & SHOW_CD_PRIMITIVES) {
		glDisable(GL_TEXTURE_2D);
		glColor4d(0.5, 0.5, 0.5, alpha);

		glEnable(GL_CULL_FACE);
		glCullFace(GL_FRONT);
		for (uint i = 0; i<cdps.size(); i++)
			cdps[i]->draw();
		glCullFace(GL_BACK);
		for (uint i = 0; i<cdps.size(); i++)
			cdps[i]->draw();
		glDisable(GL_CULL_FACE);
	}

	if (flags & SHOW_ABSTRACT_VIEW) {
		//draw capsules that define the "skeleton" of this body: parent joints TO features TO child joints and end effectors
		P3D firstFeature = (rbProperties.bodyPointFeatures.size() > 0) ? (rbProperties.bodyPointFeatures.begin()->coords) : P3D();
		P3D lastFeature = (rbProperties.bodyPointFeatures.size() > 0) ? ((rbProperties.bodyPointFeatures.end() - 1)->coords) : P3D();

		glColor3d(1, selected ? 0.5 : 1, selected ? 0.5 : 1);
		glColor3d(color[0], color[1], color[2]);
		for (uint i = 0; i < pJoints.size(); i++)
			drawCylinder(pJoints[i]->cJPos, firstFeature, abstractViewCylinderRadius, 16);

		for (int i = 0; i < (int)rbProperties.bodyPointFeatures.size() - 1; i++) {
			drawCylinder(rbProperties.bodyPointFeatures[i].coords, rbProperties.bodyPointFeatures[i + 1].coords, abstractViewCylinderRadius, 16);
			drawSphere(rbProperties.bodyPointFeatures[i].coords, abstractViewCylinderRadius * 1.2, 16);
		}

		glColor3d(1, selected ? 0.5 : 1, selected ? 0.5 : 1);
		glColor3d(color[0], color[1], color[2]);
		for (uint i = 0; i < cJoints.size(); i++)
			drawCylinder(lastFeature, cJoints[i]->pJPos, abstractViewCylinderRadius, 16);

		glColor3d(color[0], color[1], color[2]);
		glColor3d(1, selected ? 0.5 : 1, selected ? 0.5 : 1);
		for (uint i = 0; i < rbProperties.endEffectorPoints.size(); i++)
			drawCylinder(lastFeature, rbProperties.endEffectorPoints[i].coords, abstractViewCylinderRadius, 16);

		glColor3d(0, 0, 1);
		for (uint i = 0; i < cJoints.size(); i++)
			drawSphere(cJoints[i]->pJPos, abstractViewCylinderRadius*1.2, 16);

		glColor3d(1, 0, 0);
		for (uint i = 0; i < rbProperties.endEffectorPoints.size(); i++)
			drawSphere(rbProperties.endEffectorPoints[i].coords, abstractViewCylinderRadius*1.2, 16);
	}

	if (flags & SHOW_MOI_BOX) {
		glDisable(GL_TEXTURE_2D);
		glColor3d(0.5, 0.5, 0.5);
		drawMOIBox();
	}

	/* also draw a set of axes that belong to the local coordinate system*/
	/*	if (flags & SHOW_BODY_FRAME){
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	glColor3d(1, 0, 0);
	glVertex3d(0, 0, 0);
	glVertex3d(0.1, 0, 0);
	glColor3d(0, 1, 0);
	glVertex3d(0, 0, 0);
	glVertex3d(0, 0.1, 0);
	glColor3d(0, 0, 1);
	glVertex3d(0, 0, 0);
	glVertex3d(0, 0, 0.1);
	glEnd();
	}
	*/
	glPopAttrib();

	// now we'll draw the object's mesh
	if (meshes.size()>0 && (flags & SHOW_MESH)) {
		for (uint i = 0; i < meshes.size(); i++) {
			if (i < meshTransformations.size())
			{
				glPushMatrix();
				applyGLMatrixTransform(meshTransformations[i]);
				meshes[i]->drawMesh();
				glPopMatrix();
			}
			else
				meshes[i]->drawMesh();
		}
	}
	glPopAttrib();

	glPopMatrix();

	//drawing joints happens in world coordinates directly...
	if (flags & SHOW_JOINTS) {
		glColor3d(1, 1, 1);
		for (uint i = 0; i < cJoints.size(); i++)
			cJoints[i]->drawAxes();
	}

}

void RigidBody::draw(int flags, double alpha){

	draw(flags, V3D(1.0), alpha);
}

/**
	This method is used to calculate and draw boxes using Moment of Inertia
*/
void RigidBody::drawMOIBox(){
	if (rbProperties.isFrozen) return;
	drawMOIApproximation(rbProperties.MOI_local, rbProperties.mass);
}

// returns the world coords moment of inertia of the rigid body if it was to rotate about p, which is also expressed in world coords
Matrix3x3 RigidBody::getWorldMOIAboutPoint(const P3D& p){
	//TODO: this method needs more testing...

	Vector3d v = p - state.position;
	//as per the parallel axis theorem, the MOI of the body about p is: MOI_NEW = MOI_COM + m (I*v^t*v - v*v^t), where I is the identity and v is the vector from the COM of the body to the COM of the body frame
	return getWorldMOI() + (Matrix3x3::Identity() * v.dot(v) - v * v.transpose()) * rbProperties.mass;
}

Matrix3x3 RigidBody::getWorldMOI() {
	return getWorldMOI(state.orientation);
}

// returns the world coords moment of inertia of the rigid body
Matrix3x3 RigidBody::getWorldMOI(const Quaternion& orientation) {
	Matrix3x3 R = orientation.getRotationMatrix();
	return R * rbProperties.MOI_local * R.transpose();
}

/**
	This method renders the rigid body, with its current state, to OBJ that is passed in. 

	vertexIdxOffset indicates the index of the first vertex for this object, this makes it possible to render
	multiple different meshes to the same OBJ file
	 
	Returns the number of vertices written to the file
*/
uint RigidBody::renderToObjFile(FILE* fp, uint vertexIdxOffset) {
	int retVal = 0;
	for (uint i=0;i<meshes.size();i++)
		retVal += meshes[i]->renderToObjFile( fp, vertexIdxOffset+retVal, state.orientation, state.position);
	return retVal;
}

/**
	writes the RB to file
*/
void RigidBody::writeToFile(FILE* fp){
	char* str;

	fprintf(fp, "%s\n", getRBString(RB_RB));

	fprintf(fp, "\t%s %s\n", getRBString(RB_NAME), this->name.c_str());

	fprintf(fp, "\t%s %lf\n", getRBString(RB_MASS), this->rbProperties.mass);

	fprintf(fp, "\t%s %lf %lf %lf %lf %lf %lf\n", getRBString(RB_MOI), this->rbProperties.MOI_local(0,0), this->rbProperties.MOI_local(1,1), this->rbProperties.MOI_local(2,2), this->rbProperties.MOI_local(0,1), this->rbProperties.MOI_local(0,2), this->rbProperties.MOI_local(1,2));

	fprintf(fp, "\t%s %lf %lf %lf\n", getRBString(RB_POSITION), state.position[0], state.position[1], state.position[2]);

	V3D v = state.orientation.v; v.toUnit();
	fprintf(fp, "\t%s %lf %lf %lf %lf\n", getRBString(RB_ORIENTATION), state.orientation.getRotationAngle(state.orientation.v), v[0], v[1], v[2]);

	fprintf(fp, "\t%s %lf %lf %lf\n", getRBString(RB_VELOCITY), state.velocity[0], state.velocity[1], state.velocity[2]);

	fprintf(fp, "\t%s %lf %lf %lf\n", getRBString(RB_ANGULAR_VELOCITY), state.angularVelocity[0], state.angularVelocity[1], state.angularVelocity[2]);

	fprintf(fp, "\t%s %lf\n", getRBString(RB_FRICTION_COEFF), rbProperties.frictionCoeff);

	fprintf(fp, "\t%s %lf\n", getRBString(RB_RESTITUTION_COEFF), rbProperties.restitutionCoeff);

	fprintf(fp, "\t%s %lf\n", getRBString(RB_THICKNESSS), rbProperties.thickness);

	for (uint i = 0; i < meshes.size(); i++)
		fprintf(fp, "\t%s %s\n", getRBString(RB_MESH_NAME), meshes[i]->path.empty() ? "None" : meshes[i]->path.c_str());

	for (uint i = 0; i < carveMeshes.size(); i++) {
		fprintf(fp, "\t%s %s\n", getRBString(RB_CARVE_MESH_NAME), carveMeshes[i] ? carveMeshes[i]->path.c_str() : "None");
	}

	for (uint i = 0; i < meshDescriptions.size(); i++) {
		fprintf(fp, "\t%s %s\n", getRBString(RB_MESH_DESCRIPTION), meshDescriptions[i].c_str());
	}

	for (uint i = 0; i < meshTransformations.size(); i++) {
		Quaternion q;
		q.setRotationFrom(meshTransformations[i].R);
		V3D T = meshTransformations[i].T;
		fprintf(fp, "\t%s %lf %lf %lf %lf %lf %lf %lf\n", getRBString(RB_MESH_TRANSFORMATION), 
			q[0], q[1], q[2], q[3], T[0], T[1], T[2]);
	}
		

	for (uint i = 0;i<rbProperties.bodyPointFeatures.size();i++)
		fprintf(fp, "\t%s %lf %lf %lf %lf\n", getRBString(RB_BODY_POINT_FEATURE), rbProperties.bodyPointFeatures[i].coords[0], rbProperties.bodyPointFeatures[i].coords[1], rbProperties.bodyPointFeatures[i].coords[2], rbProperties.bodyPointFeatures[i].featureSize);

	for (uint i = 0;i<rbProperties.endEffectorPoints.size();i++)
		fprintf(fp, "\t%s %lf %lf %lf %lf\n", getRBString(RB_END_EFFECTOR), rbProperties.endEffectorPoints[i].coords[0], rbProperties.endEffectorPoints[i].coords[1], rbProperties.endEffectorPoints[i].coords[2], rbProperties.endEffectorPoints[i].featureSize);

	fprintf(fp, "\t%s %d %d\n", getRBString(RB_MAPPING_INFO), mappingInfo.index1, mappingInfo.index2);

	for (uint i=0;i<cdps.size();i++)
		fprintf(fp, "\t%s %s %s\n", getRBString(RB_CDP), cdps[i]->getKeyword().c_str(), cdps[i]->getDefinitionString().c_str());

	if (rbProperties.isFrozen)
		fprintf(fp, "\t%s\n", getRBString(RB_IS_FROZEN));

	str = getRBString(RB_END_RB);
	fprintf(fp, "%s\n\n\n", str);

	//TODO: skip all mesh stuff for now (RB_COLOR, RB_MATERIAL_NAME, RB_MESH_NAME)... functionally, this doesn't matter, but it would be nice to fix sometime...
}

/**
	This method loads all the pertinent information regarding the rigid body from a file.
*/
void RigidBody::loadFromFile(FILE* fp){
	if (fp == NULL)
		throwError("Invalid file pointer.");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];


	//temporary variables that we may end up populating
	GLMesh* tmpMesh;

	//this is where it happens.
	while (!feof(fp)){
		//get a line from the file...
		readValidLine(buffer, 200, fp);
		char *line = lTrim(buffer);
		int lineType = getRBLineType(line);
		switch (lineType) {
			case RB_NAME:
				this->name = std::string() + trim(line);
				break;
			case RB_MESH_NAME: {
					char tmpStr[200];
					sscanf(line, "%s", tmpStr);
					std::string str(tmpStr);
					if (str != "None") {
						tmpMesh = GLContentManager::getGLMesh(tmpStr);
						tmpMesh->computeNormals();
						tmpMesh->computeTangents();
						meshes.push_back(tmpMesh);
					}					
				}
				break;
			case RB_CARVE_MESH_NAME: {
				char tmpStr[200];
				sscanf(line, "%s", tmpStr);
				std::string str(tmpStr);
				if (str == "None"){
					carveMeshes.push_back(NULL);
				}
				else {
					tmpMesh = GLContentManager::getGLMesh(tmpStr);
					tmpMesh->computeNormals();
					tmpMesh->computeTangents();
					carveMeshes.push_back(tmpMesh);
				}				
			}
			break;
			case RB_MATERIAL:
				if (meshes.size() > 0) {
					char tmpStr[200];
					sscanf(line, "%s", tmpStr);
					GLShaderMaterial* shaderMaterial = GLContentManager::getShaderMaterial(tmpStr);
					if (shaderMaterial == NULL)
						Logger::consolePrint("RigidBody::loadFromFile: specified material not found...\n");
					else
						meshes[meshes.size() - 1]->setMaterial(*shaderMaterial);
				}
				else
					Logger::consolePrint("RigidBody::loadFromFile: Warning, specified material does not apply to any mesh\n");
				break;
			case RB_COLOR: {
					double r, g, b, a;
					if (sscanf(line, "%lf %lf %lf %lf", &r, &g, &b, &a) != 4)
						throwError("Incorrect rigid body input file - colour parameter expects 4 arguments (colour %s)\n", line);
					if (meshes.size() > 0)
						meshes[meshes.size() - 1]->getMaterial().setColor(r, g, b, a);
				}
				break;
			case RB_MASS: {
					double t = 1;
					if (sscanf(line, "%lf", &t) != 1)
						Logger::consolePrint("Incorrect rigid body input file - a mass needs to be specified if the 'mass' keyword is used.");
					this->rbProperties.mass = t;
				}
				break;
			case RB_MOI: {
					double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0;
					sscanf(line, "%lf %lf %lf %lf %lf %lf", &t1, &t2, &t3, &t4, &t5, &t6);
					if (t1 <= 0 || t2 <= 0 || t3 <= 0)
						Logger::consolePrint("Incorrect values for the principal moments of inertia.");
					this->rbProperties.setMOI(t1, t2, t3, t4, t5, t6);
				}
				break;
			case RB_POSITION:
				if (sscanf(line, "%lf %lf %lf", &state.position[0], &state.position[1], &state.position[2]) != 3)
					throwError("Incorrect rigid body input file - 3 arguments are required to specify the world coordinates position of a rigid body\n", line);
				break;
			case RB_ORIENTATION: {
					double t = 0, t1 = 0, t2 = 0, t3 = 0;
					if (sscanf(line, "%lf %lf %lf %lf", &t, &t1, &t2, &t3) != 4)
						throwError("Incorrect rigid body input file - 4 arguments are required to specify the world coordinates orientation of a rigid body\n", line);
					state.orientation.setRotationFrom(t, V3D(t1, t2, t3).toUnit());
				}
				break;
			case RB_VELOCITY:
				if (sscanf(line, "%lf %lf %lf", &state.velocity[0], &state.velocity[1], &state.velocity[2]) != 3)
					throwError("Incorrect rigid body input file - 3 arguments are required to specify the world coordinates velocity of a rigid body\n", line);
				break;
			case RB_ANGULAR_VELOCITY:
				if (sscanf(line, "%lf %lf %lf", &state.angularVelocity[0], &state.angularVelocity[1], &state.angularVelocity[2]) != 3)
					throwError("Incorrect rigid body input file - 3 arguments are required to specify the world coordinates angular velocity of a rigid body\n", line);
				break;
			case RB_FRICTION_COEFF:
				if (sscanf(line, "%lf", &rbProperties.frictionCoeff) != 1)
					throwError("Incorrect rigid body input file - Expecting a value for the friction coefficient");
				if (rbProperties.frictionCoeff<0)
					throwError("Incorrect rigid body input file - Friction coefficient should be >= 0");
				break;
			case RB_RESTITUTION_COEFF:
				if (sscanf(line, "%lf", &rbProperties.restitutionCoeff) != 1)
					throwError("Incorrect rigid body input file - Expecting a value for the restitution coefficient");
				if (rbProperties.restitutionCoeff<0 || rbProperties.restitutionCoeff>1)
					throwError("Incorrect rigid body input file - restitution coefficient should be between 0 and 1");
				break;
			case RB_END_EFFECTOR:{
				P3D p;
				double featureSize = 0.02;
				sscanf(line, "%lf %lf %lf %lf", &p[0], &p[1], &p[2], &featureSize);
//				if (sscanf(line, "%lf %lf %lf", &p[0], &p[1], &p[2]) != 3)
//					throwError("Incorrect rigid body input file - 3 arguments are required to specify a body point feature\n", line);
					rbProperties.endEffectorPoints.push_back(RBEndEffector(p, featureSize));
				}
				break;
			case RB_BODY_POINT_FEATURE: {
					P3D p;
					double featureSize = 0.02;
					sscanf(line, "%lf %lf %lf %lf", &p[0], &p[1], &p[2], &featureSize);
//					if (sscanf(line, "%lf %lf %lf", &p[0], &p[1], &p[2], &featureSize) != 3)
//						throwError("Incorrect rigid body input file - 3 arguments are required to specify a body point feature\n", line);
					rbProperties.bodyPointFeatures.push_back(RBFeaturePoint(p, featureSize));
				}
				break;
			case RB_NOT_IMPORTANT:
				if (strlen(line) != 0 && line[0] != '#')
					Logger::consolePrint("Ignoring input line: \'%s\'\n", line);
				break;
			case RB_CDP: {
					CollisionDetectionPrimitive* newCDP = CollisionDetectionPrimitive::getCDPFromDefinition(lTrim(line));
					if (newCDP == NULL)
						Logger::consolePrint("Could not load CDP from definition string: %s\n", line);
					else
						cdps.push_back(newCDP);
				}
				break;
			case RB_IS_FROZEN:
				this->rbProperties.isFrozen = true;
				break;
			case RB_END_RB:
				if (meshDescriptions.size() != meshes.size())
					meshDescriptions.resize(meshes.size());
				return;//and... done
				break;
			case RB_THICKNESSS:
				if (sscanf(line, "%lf", &rbProperties.thickness) != 1)
					throwError("Incorrect rigid body input file - Expecting a value for the thickness coefficient");
				break;
			case RB_MESH_DESCRIPTION: {
				char tmpStr[200];
				sscanf(line, "%s", tmpStr);
				std::string str(tmpStr);
				meshDescriptions.push_back(str);
			}
				break;
			case RB_MESH_TRANSFORMATION: {
				Quaternion q;
				V3D T;
				sscanf(line, "%lf %lf %lf %lf %lf %lf %lf",
					&q[0], &q[1], &q[2], &q[3], &T[0], &T[1], &T[2]);
				meshTransformations.push_back(Transformation(q.getRotationMatrix(), T));
				break;
			}
			case RB_MAPPING_INFO:
				sscanf(line, "%d %d", &mappingInfo.index1, &mappingInfo.index2);
				break;
			default:
				throwError("Incorrect rigid body input file: \'%s\' - unexpected line.", buffer);
		}
	}
	throwError("Incorrect articulated body input file! No /End found");
}

#define UPDATE_RAY_INTERSECTION(P1, P2)														\
if (localCoordsRay.getDistanceToSegment(P1, P2, &tmpIntersectionPoint) < cylRadius) {		\
	double tVal = localCoordsRay.getRayParameterFor(tmpIntersectionPoint);					\
	if (tVal < tMin) {																		\
		*localCoordsIntersectionPoint = tmpIntersectionPoint;								\
		tMin = tVal;																		\
	}																						\
}												

void RigidBody::autoGenerateCDPs() {
	P3D tmpIntersectionPoint;
	double tMin = DBL_MAX;
	double cylRadius = abstractViewCylinderRadius;

	//draw capsules that define the "skeleton" of this body: parent joints TO features TO child joints and end effectors
	P3D firstFeature = (rbProperties.bodyPointFeatures.size() > 0) ? (rbProperties.bodyPointFeatures.begin()->coords) : P3D();
	P3D lastFeature = (rbProperties.bodyPointFeatures.size() > 0) ? ((rbProperties.bodyPointFeatures.end() - 1)->coords) : P3D();

	for (uint i = 0; i < pJoints.size(); i++)
		cdps.push_back(new CapsuleCDP(pJoints[i]->cJPos, firstFeature, cylRadius));

	for (int i = 0; i < (int)rbProperties.bodyPointFeatures.size() - 1; i++)
		cdps.push_back(new CapsuleCDP(rbProperties.bodyPointFeatures[i].coords, rbProperties.bodyPointFeatures[i + 1].coords, cylRadius));

	for (uint i = 0; i < cJoints.size(); i++)
		cdps.push_back(new CapsuleCDP(lastFeature, cJoints[i]->pJPos, cylRadius));

	for (uint i = 0; i < rbProperties.endEffectorPoints.size(); i++)
		cdps.push_back(new CapsuleCDP(lastFeature, rbProperties.endEffectorPoints[i].coords, cylRadius));

}


/*returns true if it is hit, false otherwise. */
bool RigidBody::getRayIntersectionPointTo(const Ray& ray, P3D* localCoordsIntersectionPoint){
	P3D tmpIntersectionPoint;
	double tMin = DBL_MAX;
	double cylRadius = abstractViewCylinderRadius;
	//the ray is in world coords, so change it to body coords...
	Ray localCoordsRay = Ray(getLocalCoordinates(ray.origin), getLocalCoordinates(ray.direction));

	//draw capsules that define the "skeleton" of this body: parent joints TO features TO child joints and end effectors
	P3D firstFeature = (rbProperties.bodyPointFeatures.size() > 0) ? (rbProperties.bodyPointFeatures.begin()->coords) : P3D();
	P3D lastFeature = (rbProperties.bodyPointFeatures.size() > 0) ? ((rbProperties.bodyPointFeatures.end() - 1)->coords) : P3D();

	for (uint i = 0; i < pJoints.size(); i++)
		UPDATE_RAY_INTERSECTION(pJoints[i]->cJPos, firstFeature);

	for (int i = 0; i < (int)rbProperties.bodyPointFeatures.size() - 1; i++)
		UPDATE_RAY_INTERSECTION(rbProperties.bodyPointFeatures[i].coords, rbProperties.bodyPointFeatures[i + 1].coords);

	for (uint i = 0; i < cJoints.size(); i++)
		UPDATE_RAY_INTERSECTION(lastFeature, cJoints[i]->pJPos);

	for (uint i = 0; i < rbProperties.endEffectorPoints.size(); i++)
		UPDATE_RAY_INTERSECTION(lastFeature, rbProperties.endEffectorPoints[i].coords);

	return tMin < DBL_MAX / 2.0;
}
