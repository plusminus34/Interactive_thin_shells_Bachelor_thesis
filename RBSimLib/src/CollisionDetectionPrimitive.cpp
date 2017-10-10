#include <RBSimLib/CollisionDetectionPrimitive.h>
#include <GUILib/GLUtils.h>


std::string SphereCDP::keyword = "Sphere";
std::string PlaneCDP::keyword = "Plane";
std::string BoxCDP::keyword = "Box";
std::string CapsuleCDP::keyword = "Capsule";

CollisionDetectionPrimitive::CollisionDetectionPrimitive(){
}

CollisionDetectionPrimitive::~CollisionDetectionPrimitive(void){
}

/**
	draw an outline of the primitive...
*/
void CollisionDetectionPrimitive::draw(){
}

CollisionDetectionPrimitive* CollisionDetectionPrimitive::getCDPFromDefinition(const std::string& def) {
	std::vector<std::string> keywords;
	getCharSeparatedStringList(def.c_str(), keywords, ' ');

	if (keywords[0].compare(SphereCDP::keyword) == 0)
		return new SphereCDP(def.c_str() + SphereCDP::keyword.length());

	if (keywords[0].compare(PlaneCDP::keyword) == 0)
		return new PlaneCDP(def.c_str() + PlaneCDP::keyword.length());

	if (keywords[0].compare(BoxCDP::keyword) == 0)
		return new BoxCDP(def.c_str() + BoxCDP::keyword.length());

	if (keywords[0].compare(CapsuleCDP::keyword) == 0)
		return new CapsuleCDP(def.c_str() + CapsuleCDP::keyword.length());

	return NULL;
}

SphereCDP::SphereCDP() { }

SphereCDP::SphereCDP(const SphereCDP& other) {
	this->p = other.p; this->r = other.r;
}

SphereCDP::SphereCDP(const P3D& p, double r) {
	this->p = p; this->r = r;
}


SphereCDP::SphereCDP(const std::string& def) {
	if (sscanf(def.c_str(), "%lf %lf %lf %lf", &p[0], &p[1], &p[2], &r) != 4)
		throwError("Incorrect SPhereCDP definition: 4 arguments are required...\n", def.c_str());
}

void SphereCDP::draw() {
	drawSphere(p, r);
}

std::string SphereCDP::getDefinitionString() {
	return std::to_string(p[0]) + " " + std::to_string(p[1]) + " " + std::to_string(p[2]) + " " + std::to_string(r);
}

//------------------------------------------------------------------------------------------------------------------\\

PlaneCDP::PlaneCDP() { }

PlaneCDP::PlaneCDP(const PlaneCDP& other) {
	this->p = other.p;
}

PlaneCDP::PlaneCDP(const Plane& p) {
	this->p = p;
}


PlaneCDP::PlaneCDP(const std::string& def) {
	if (sscanf(def.c_str(), "%lf %lf %lf %lf %lf %lf", &p.n[0], &p.n[1], &p.n[2], &p.p[0], &p.p[1], &p.p[2]) != 6)
		throwError("Incorrect PlaneCDP definition: 6 arguments are required...\n", def.c_str());
}

void PlaneCDP::draw() {
	//not going to draw an infinite plane...
}

std::string PlaneCDP::getDefinitionString() {
	return std::to_string(p.n[0]) + " " + std::to_string(p.n[1]) + " " + std::to_string(p.n[2]) + " " +
		std::to_string(p.p[0]) + " " + std::to_string(p.p[1]) + " " + std::to_string(p.p[2]);
}

//------------------------------------------------------------------------------------------------------------------\\

BoxCDP::BoxCDP() { }

BoxCDP::BoxCDP(const BoxCDP& other) {
	this->p1 = other.p1;
	this->p2 = other.p2;
}

BoxCDP::BoxCDP(const P3D& p1, const P3D& p2) {
	this->p1 = p1;
	this->p2 = p2;
}

BoxCDP::BoxCDP(const std::string& def) {
	if (sscanf(def.c_str(), "%lf %lf %lf %lf %lf %lf", &p1[0], &p1[1], &p1[2], &p2[0], &p2[1], &p2[2]) != 6)
		throwError("Incorrect BoxCDP definition: 6 arguments are required...\n", def.c_str());
}

void BoxCDP::draw() {
	drawBox(p1, p2);
}

std::string BoxCDP::getDefinitionString() {
	return std::to_string(p1[0]) + " " + std::to_string(p1[1]) + " " + std::to_string(p1[2]) + " " +
		std::to_string(p2[0]) + " " + std::to_string(p2[1]) + " " + std::to_string(p2[2]);
}


//------------------------------------------------------------------------------------------------------------------\\

CapsuleCDP::CapsuleCDP() { }

CapsuleCDP::CapsuleCDP(const CapsuleCDP& other) {
	this->p1 = other.p1;
	this->p2 = other.p2;
	this->r = other.r;
}

CapsuleCDP::CapsuleCDP(const P3D& p1, const P3D& p2, double r) {
	this->p1 = p1;
	this->p2 = p2;
	this->r = r;
}

CapsuleCDP::CapsuleCDP(const std::string& def) {
	if (sscanf(def.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &p1[0], &p1[1], &p1[2], &p2[0], &p2[1], &p2[2], &r) != 7)
		throwError("Incorrect CapsuleCDP definition: 7 arguments are required...\n", def.c_str());
}

void CapsuleCDP::draw() {
	drawCapsule(p1, p2, r);
}

std::string CapsuleCDP::getDefinitionString() {
	return std::to_string(p1[0]) + " " + std::to_string(p1[1]) + " " + std::to_string(p1[2]) + " " +
		std::to_string(p2[0]) + " " + std::to_string(p2[1]) + " " + std::to_string(p2[2]) + " " + std::to_string(r);
}

