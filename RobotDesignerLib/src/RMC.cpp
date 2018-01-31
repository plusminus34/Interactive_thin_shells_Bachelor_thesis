#include <RobotDesignerLib/RMC.h>
#include <GUILib/GLApplication.h>
#include <RobotDesignerLib/RMCBulletObject.h>

RMC::RMC(GLMesh* mesh){
	this->meshes.push_back(mesh);
}

RMC::~RMC(){
}

void RMC::copyBasePropertiesTo(RMC* other, bool includePinInfo) {
	other->state = state;
	other->rbProperties = rbProperties;

	other->meshes = meshes;
	other->name = name;
	other->id = id;
	other->type = type;

	other->material = material;

	other->mappingInfo = mappingInfo;

	if (includePinInfo){
		other->pins = pins;
		for (uint i = 0; i < pins.size(); i++)
			other->pins[i].rmc = other;
	}
}

RMC* RMC::clone(){
	RMC* new_rmc = new RMC();
	copyBasePropertiesTo(new_rmc, true);
	return new_rmc;
}

bool RMC::isConnected(RMC* rmc) {

	return (getParentJoint() && getParentJoint()->getParent() == rmc) || (rmc->getParentJoint() && rmc->getParentJoint()->getParent() == this);
}

bool RMC::isMovable(){
	if (pJoints.empty()) return true;

	for (auto pJoint : pJoints)	{
		RMC* rmc = dynamic_cast<RMC*>(pJoint->parent);
		if (rmc->type != LIVING_CONNECTOR) return false;
	}

	return true;
}

bool RMC::pickPin(Ray& ray){
	pickedPin = NULL;
	for (uint i = 0; i < pins.size(); i++)
	{
		if (pins[i].idle && pins[i].isPicked(ray))
		{
			pickedPin = &pins[i];
			return true;
		}
	}

	return false;
}

bool RMC::pickMesh(Ray& ray, double* closestDist){
	Transformation invTrans = Transformation(state.orientation.getRotationMatrix(), state.position).inverse();
	Ray newRay(invTrans.transform(ray.origin), invTrans.transform(ray.direction));

	return meshes[0]->getDistanceToRayOriginIfHit(newRay, closestDist);
}

void RMC::drawPins() {
	for (uint i = 0; i < pins.size(); i++) {
		if ((&pins[i]) == pickedPin)
			pins[i].draw(V3D(1, 0, 0));
		else
			pins[i].draw(V3D(0, 1, 1));
	}

}


void RMC::draw(int flags, const Vector4d& color){
	glEnable(GL_LIGHTING);
	if (flags & SHOW_PINS)
		drawPins();
	if (color.isZero()){
		meshes[0]->setMaterial(material);
	}
	else {
		GLShaderMaterial colorMat;
		colorMat.setColor(color[0], color[1], color[2], color[3]);
		meshes[0]->setMaterial(colorMat);
	}

	RigidBody::draw(flags);
}

//should return true if it's parsed it, false otherwise...
bool RMC::interpretInputLine(FILE* fp, char* line) {
	char keyword[50];
	char content[200];

	sscanf(line, "%s", keyword);

	if (strcmp(keyword, "Name") == 0){
		sscanf(line + strlen(keyword), "%s", content);
		name = content;
		return true;
	}
	else if (strcmp(keyword, "Plate") == 0){
		type = PLATE_RMC;
		return true;
	}
	else if (strcmp(keyword, "EE") == 0){
		type = EE_RMC;
		return true;
	}
	else if (strcmp(keyword, "EndEffector") == 0){
		type = EE_RMC;
		P3D ee;
		sscanf(line + strlen(keyword), "%lf %lf %lf", &ee[0], &ee[1], &ee[2]);
		rbProperties.endEffectorPoints.push_back(ee);
		return true;
	}
	else if (strcmp(keyword, "FeaturePoint") == 0){
		P3D FP;
		sscanf(line + strlen(keyword), "%lf %lf %lf", &FP[0], &FP[1], &FP[2]);
		rbProperties.bodyPointFeatures.push_back(FP);
		return true;
	}
	else if (strcmp(keyword, "Mass") == 0){
		sscanf(line + strlen(keyword), "%lf", &rbProperties.mass);
		return true;
	}
	else if (strcmp(keyword, "MOI") == 0){
		Matrix3x3& MOI = rbProperties.MOI_local;
		int num = sscanf(line + strlen(keyword), "%lf %lf %lf %lf %lf %lf", &MOI(0, 0), &MOI(1, 1), &MOI(2, 2)
			, &MOI(0, 1), &MOI(0, 2), &MOI(1, 2));
		if (num < 6)
			throwError("Not enough transformation parameters!");

		return true;
	}
	else if (strcmp(keyword, "Mesh") == 0){
		sscanf(line + strlen(keyword), "%s", content);
		GLMesh* mesh = GLContentManager::getGLMesh(content);
		mesh->calBoundingBox();
		mesh->getMaterial().setColor(1.0, 1.0, 1.0, 0.8);
		meshes.push_back(mesh);
		return true;
	}
	else if (strcmp(keyword, "Material") == 0){
		sscanf(line + strlen(keyword), "%s", content);
		material.setShaderProgram(GLContentManager::getShaderProgram("matcap"));
		material.setTextureParam(content, GLContentManager::getTexture(content));
		return true;
	}
	else if (strcmp(keyword, "Pin") == 0){
		pins.push_back(RMCPin(this, (int)pins.size()));
		pins.back().loadFromFile(fp);
	}

	return false;
}


void RMC::loadFromFile(FILE* fp){
	char buffer[200];

	RMCBulletObject* bulletObject = NULL;

	while (!feof(fp)) {
		//get a line from the file...
		readValidLine(buffer, fp, 200);
		if (strlen(buffer) > 195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		if (strlen(line) == 0) continue;

		if (interpretInputLine(fp, line))
			continue;

		if (strncmp(line, "EndRMC", strlen("EndRMC")) == 0)
			break;

		Logger::print("Ignoring input line \'%s\'\n", line);
	}
}
