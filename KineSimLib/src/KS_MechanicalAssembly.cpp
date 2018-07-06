#include "KineSimLib/KS_MechanicalAssembly.h"
#include "KineSimLib/KS_LoaderUtils.h"
//#include "KineSimLib/KS_Gear.h"
//#include "KineSimLib/KS_Phase2PhaseConnection.h"
//#include "KineSimLib/KS_PhaseDriverConnection.h"
//#include "KineSimLib/KS_Gear2GearConnection.h"
#include "KineSimLib/KS_BindComponentsConnection.h"
#include "KineSimLib/KS_BoundToWorldConnection.h"
#include "KineSimLib/KS_PointOnLineConnection.h"
//#include "KineSimLib/KS_MotorConnection.h"
//#include "KineSimLib/KS_Bar.h"
//#include "KineSimLib/KS_Shaft.h"
#include "KineSimLib/KS_GenericComponent.h"
//#include "KineSimLib/KS_NonCircularGear.h"
//#include "KineSimLib/KS_NonCircularGearsConnection.h"
//#include "KineSimLib/KS_Quad.h"
//#include "KineSimLib/KS_MultiLinkBar.h"
//#include "KineSimLib/KS_HermiteSplineLinkBar.h"
#include "KineSimLib/KS_PlanarComponentConnection.h"
#include <GUILib/GLUtils.h>
#include <string>
#include <sstream>
#include <iostream>
#include <OptimizationLib/NewtonFunctionMinimizer.h>
#include <OptimizationLib/BFGSFunctionMinimizer.h>
#include <OptimizationLib/CMAFunctionMinimizer.h>

using namespace std;

KS_MechanicalAssembly::KS_MechanicalAssembly(void){
	m_ticker = KS_Ticker();
	AConstraintEnergy = NULL;
	
}

KS_MechanicalAssembly::KS_MechanicalAssembly(KS_MechanicalAssembly &a){
	m_components.resize(a.getComponents().size());
	m_connections.resize(a.getConnections().size());
	//m_components.clear();
	//m_connections.clear();
	for(uint i=0;i<m_components.size();i++){
		m_components[i] = a.getComponent(i)->clone();
	}
	for(uint i=0;i<m_connections.size();i++){
		KS_MechanicalComponent* mCompIn = NULL;
		KS_MechanicalComponent* mCompOut = NULL;
		if(a.getConnection(i)->getInput()!=NULL)
			mCompIn=this->getComponent(a.getConnection(i)->getInput()->getComponentIndex());
		if(a.getConnection(i)->getOutput()!=NULL)
			mCompOut=this->getComponent(a.getConnection(i)->getOutput()->getComponentIndex());		
		m_connections[i] = a.getConnection(i)->clone(mCompIn,mCompOut,&m_ticker);
	}
	m_ticker.setTickerValue(a.getTickerValue());
}

KS_MechanicalAssembly::~KS_MechanicalAssembly(void){
	ComponentIt it1 = m_components.begin();
	for(it1; it1!=m_components.end();it1++)
		delete *it1;

	ConnectionIt it2 = m_connections.begin();
	for(it2; it2!=m_connections.end();it2++)
		delete *it2;
	delete AConstraintEnergy;
}

void KS_MechanicalAssembly::getAssemblyState(dVector& state){
	int stateSize = getComponentCount()*KS_MechanicalComponent::getStateSize();
	state.resize(stateSize);
	for (int i=0;i<getComponentCount();i++){
		KS_MechanicalComponent* c = getComponent(i);
		int i0 = i* KS_MechanicalComponent::getStateSize();
		state[i0+0] = c->getGamma();
		state[i0+1] = c->getBeta();
		state[i0+2] = c->getAlpha();
		state[i0+3] = c->getWorldCenterPosition()[0];
		state[i0+4] = c->getWorldCenterPosition()[1];
		state[i0+5] = c->getWorldCenterPosition()[2];
	}

}

void KS_MechanicalAssembly::setAssemblyState(const dVector& state){
	for (int i=0;i<getComponentCount();i++){
		KS_MechanicalComponent* c = getComponent(i);
		c->setAngles(state[KS_MechanicalComponent::getStateSize() * i + 0], state[KS_MechanicalComponent::getStateSize() * i + 1], state[KS_MechanicalComponent::getStateSize() * i + 2]);
		c->setWorldCenterPosition(P3D(state[KS_MechanicalComponent::getStateSize() * i + 3], state[KS_MechanicalComponent::getStateSize() * i + 4], state[KS_MechanicalComponent::getStateSize() * i + 5]));
	}
}


/**
	This method is used to load the details of a mechanical assembly from file.
*/
bool KS_MechanicalAssembly::readFromFile(const char* szFile){

	 m_components.clear();
     m_connections.clear();


	FILE* f = fopen(szFile, "r");
	if (f == NULL){
		Logger::print("KS_MechanicalAssembly: Cannot load input file \'%s\'\n", szFile);
		return false;
	}

	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	//this is where it happens.

	while (!feof(f)){
		//get a line from the file...
		readValidLine(buffer, f, sizeof(buffer));
		char *line = lTrim(buffer);
		int lineType = getKSLineType(line);
		switch (lineType) {
			case KS_ASSEMBLY_TICK_COUNT:{
				if(sscanf(line, "%lf", &m_ticker.m_stepPhase) != 1) assert(false);
				}
				break;
			/*case KS_SPUR_GEAR:{
					KS_Gear* gear = new KS_Gear(trim(line));
					if (!gear->loadFromFile(f))
					{
						fclose(f);
						return false;
					}
					if(!addComponent(gear))
						delete gear;
				}break;
			case KS_NON_CIRCULAR_GEAR:{
					KS_NonCircularGear* ncGear = new KS_NonCircularGear(trim(line));
					if (!ncGear->loadFromFile(f))
					{
						fclose(f);
						return false;
					}
					if(!addComponent(ncGear))
						delete ncGear;
				}break;
			case KS_BAR:{
					KS_Bar* bar = new KS_Bar(trim(line));
					if (!bar->loadFromFile(f))
					{
						fclose(f);
						return false;
					}
					if(!addComponent(bar))
						delete bar;
				}break;
			case KS_MULTI_LINK_BAR:{
					KS_MultiLinkBar* mbar = new KS_MultiLinkBar(trim(line));
					if (!mbar->loadFromFile(f))
					{
						fclose(f);
						return false;
					}
					if(!addComponent(mbar))
						delete mbar;
				}break;	
			case KS_HERMITE_SPLINE_LINK_BAR:{
				KS_HermiteSplineLinkBar* mbar = new KS_HermiteSplineLinkBar(trim(line));
				if (!mbar->loadFromFile(f))
				{
					fclose(f);
					return false;
				}
				if(!addComponent(mbar))
					delete mbar;
				}break;	
			case KS_QUAD:{
					KS_Quad* quad = new KS_Quad(trim(line));
					if (!quad->loadFromFile(f))
					{
						fclose(f);
						return false;
					}
					if(!addComponent(quad))
						delete quad;
				}break;*/
			case KS_GENERIC_COMPONENT:{
					KS_GenericComponent* gc = new KS_GenericComponent(trim(line));
					if (!gc->loadFromFile(f))
					{
						fclose(f);
						return false;
					}
					if(!addComponent(gc))
						delete gc;
				}break;
			/*case KS_SHAFT:{
					KS_Shaft* shaft = new KS_Shaft(trim(line));
					if (!shaft->loadFromFile(f))
					{
						fclose(f);
						return false;
					}
					if(!addComponent(shaft))
						delete shaft;
				}break;
			case KS_PHASE_DRIVER:{
					Logger::print("KS_MechanicalAssembly - Warning: Phase Drivers are no longer used! PhaseDriverConnections connect to the ticker directly.\n");
					Logger::printStatic("KS_MechanicalAssembly - Warning: Phase Drivers are no longer used! PhaseDriverConnections connect to the ticker directly.\n");
					while (!feof(f)){
						//get a line from the file...
						readValidLine(buffer, f, sizeof(buffer));
						char *line = lTrim(buffer);
						int lineType = getKSLineType(line);
						if (lineType == KS_END)
							break;
					}
				}break;
			case KS_PHASE_DRIVER_CON:{
					KS_PhaseDriverConnection* pdC = new KS_PhaseDriverConnection(&m_ticker);
					pdC->loadFromFile(f, this);
					this->addConnection(pdC);
				}break;
			case KS_GEAR_2_GEAR_CON:{
					KS_Gear2GearConnection* g2gC = new KS_Gear2GearConnection();
					g2gC->loadFromFile(f, this);
					this->addConnection(g2gC);
				}break;
			case KS_NON_CIRCULAR_GEARS_CON:{
					KS_NonCircularGearsConnection* g2gC = new KS_NonCircularGearsConnection();
					g2gC->loadFromFile(f, this);
					this->addConnection(g2gC);
				}break;*/
			case KS_POINT_ON_LINE_CON:{
					KS_PointOnLineConnection* p2lC = new KS_PointOnLineConnection();
					p2lC->loadFromFile(f, this);
					this->addConnection(p2lC);
				}break;
			case KS_BIND_COMPONENTS_CON:{
					KS_BindComponentsConnection* bcC = new KS_BindComponentsConnection();
					bcC->loadFromFile(f, this);
					this->addConnection(bcC);
				}break;
			/*case KS_MOTOR_CON:{
					KS_MotorConnection* mC = new KS_MotorConnection();
					mC->loadFromFile(f, this);
					this->addConnection(mC);
				}break;*/
			case KS_BOUND_TO_WORLD_CON:{
					KS_BoundToWorldConnection* bwC = new KS_BoundToWorldConnection();
					bwC->loadFromFile(f, this);
					this->addConnection(bwC);
				}break;
			case KS_PLANAR_COMP_CON:{
					KS_PlanarComponentConnection* pcC = new KS_PlanarComponentConnection();
					pcC->loadFromFile(f, this);
					this->addConnection(pcC);
				}break;
			case KS_NOT_IMPORTANT:
				if (strlen(trim(buffer)) > 0)
					Logger::print("KS_MechanicalAssembly warning: Ignoring input line: \'%s\'\n", buffer);
				break;
			case KS_COMMENT:
				break;
			default:
				Logger::print("Incorrect KS input file. Unexpected line: %s\n", buffer);
				fclose(f);
				return false;
		}
	}

	fclose(f);
	initConstraints();
	s.resize(6 * getComponentCount()); //s.setZero();
	sSolver.resize(6 * getComponentCount()); //sSolver.setZero();
	Logger::print(" number of components %d\n", getComponentCount());
	Logger::print(" number of connections %d\n", getConnectionCount());
	for (int i = 0; i < m_components.size(); i++) {
		m_components[i]->setupGeometry();
		s[KS_MechanicalComponent::getStateSize() * i + 0]= m_components[i]->getGamma(); 
	    s[KS_MechanicalComponent::getStateSize() * i + 1]= m_components[i]->getBeta(); 
		s[KS_MechanicalComponent::getStateSize() * i + 2]= m_components[i]->getAlpha(); 
		s[KS_MechanicalComponent::getStateSize() * i + 3]= m_components[i]->getWorldCenterPosition()[0];
		s[KS_MechanicalComponent::getStateSize() * i + 4]= m_components[i]->getWorldCenterPosition()[1];
	    s[KS_MechanicalComponent::getStateSize() * i + 5]= m_components[i]->getWorldCenterPosition()[2];
		//P3D temp = m_components[i]->getWorldCenterPosition();
		//Logger::print("components i x y z %d %lf %lf %lf\n", i, temp[0], temp[1], temp[2]);
	}

	AConstraintEnergy = new KS_AssemblyConstraintEnergy();
	AConstraintEnergy->initialize(this);
	return true;
}

void KS_MechanicalAssembly::writeToFile(const char* szFile){
	FILE* f = fopen(szFile, "w");
	if (f == NULL){
		Logger::print("KS_MechanicalAssembly: Cannot open input file \'%s\' for writing\n", szFile);
		return;
	}

	char* str = getKSString(KS_ASSEMBLY_TICK_COUNT);
	fprintf(f, "%s %lf\n \n", str, m_ticker.tick());

	for (uint i=0;i<m_components.size();i++)
		m_components[i]->writeToFile(f);

	for (uint i=0;i<m_connections.size();i++)
		m_connections[i]->writeToFile(f);

	fclose(f);
}
bool KS_MechanicalAssembly::addComponent(KS_MechanicalComponent* pComp){
	pComp->setComponentIndex((int)m_components.size());
	m_components.push_back(pComp);

	return true;
}

void KS_MechanicalAssembly::removeConnection(uint i){
	if(i<m_connections.size()){
		m_connections[i]->getOutput()->removeMesh(m_connections[i]->getPin());
		for(uint j=0;j<m_connections.size();j++)
			if((m_connections[j]->getOutput()==m_connections[i]->getOutput())&&(m_connections[j]->getPin()>m_connections[i]->getPin()))
				m_connections[j]->setPin(m_connections[j]->getPin()-1);
		delete(m_connections[i]);
		m_connections.erase(m_connections.begin()+i);
	}
}

void KS_MechanicalAssembly::removeComponent(uint i){
	if(i<m_components.size()){
		for(uint k=0;k<m_connections.size();k++)
			if((m_connections[k]->getInput()==m_components[i])||(m_connections[k]->getOutput()==m_components[i])){
				removeConnection(k);
				k--;
			}
		delete(m_components[i]);
		for(uint j=i+1;j<m_components.size();j++){
			m_components[j]->setComponentIndex((int)(j-1));
			KS_GenericComponent* comp = dynamic_cast<KS_GenericComponent*>(m_components[j]);
			if(comp){
				char name [200];
				std::sprintf(name,"genComp%d",j);
				comp->setName(name);
				for(uint k=0;k<m_connections.size();k++){
					if((m_connections[k]->getInput()==m_components[j])||(m_connections[k]->getOutput()==m_components[j]))
						m_connections[k]->assignConnectedComponents(m_connections[k]->getInput(),m_connections[k]->getOutput());
				}
			}
		}
		m_components.erase(m_components.begin()+i);
	}
}

void KS_MechanicalAssembly::addConnection(KS_Connection* pConn){
	pConn->linkWithTicker(&m_ticker);
	m_connections.push_back(pConn);
}

void KS_MechanicalAssembly::updateTracerParticles(){
	for (uint i = 0; i<m_components.size(); i++)
		m_components[i]->updateTracerParticles();
}

void KS_MechanicalAssembly::drawTracerParticles(){
	for (uint i = 0; i<m_components.size(); i++)
		m_components[i]->drawTracerParticles();
}

void KS_MechanicalAssembly::stepAssembly(bool updateDriver){
	if (updateDriver)
		m_ticker.step();
	for (uint i=0;i<m_connections.size();i++)
		m_connections[i]->updateConnection();
}

void KS_MechanicalAssembly::restartTicker(){
	m_ticker.restart();
	for(uint i=0;i<m_connections.size();i++)
		m_connections[i]->updateConnection();
}

void KS_MechanicalAssembly::clearTracerParticles(){
	for (uint i=0;i<this->m_components.size();i++){
		m_components[i]->clearTracerParticles();
	}
}

void KS_MechanicalAssembly::getTracerParticles(DynamicArray<P3D>& tracerParticleList){
	tracerParticleList.clear();
	for (uint i=0;i<this->m_components.size();i++)
		m_components[i]->addTracerParticlesToList(tracerParticleList);
}

void KS_MechanicalAssembly::draw(){
	for (uint i=0;i<this->m_components.size();i++){
		//GLUtils::glLColor(0.5, 0.5, 0.5);
		m_components[i]->draw();
	}
}

void KS_MechanicalAssembly::initConstraints(){
	for(uint i=0; i<m_connections.size();i++) {
		std::vector<KS_Constraint*> constraints;
		m_connections[i]->addConstraintsToList(constraints);
		for(uint j=0; j<constraints.size();j++)	{
			std::vector<int> vinds;
			KS_Constraint* c = constraints[j];
			int nc = c->getNumberOfAffectedComponents();
			for(int k=0; k<nc;k++) {
				KS_MechanicalComponent* comp = c->getIthAffectedComponent(k);
				int ibase = comp->getComponentIndex();
				for(int l = 0; l<KS_MechanicalComponent::getStateSize();l++)
					vinds.push_back(ibase*KS_MechanicalComponent::getStateSize()+l);
			}
		}
	}
}

/*KS_MechanicalComponent* KS_MechanicalAssembly::getFirstComponentIntersectedByRay(const Ray& ray){
	double minDistance = DBL_MAX;
	KS_MechanicalComponent* closestComponent = NULL;
	for (uint i=0;i<m_components.size();i++){
		P3D p;
		if (m_components[i]->isIntersectedByRay(ray, p)){
			double dist = V3D(ray.origin, p).length();
			if (dist < minDistance){
				minDistance = dist;
				closestComponent = m_components[i];
			}
		}
	}

	return closestComponent;
}*/


/*KS_MechanicalComponent* KS_MechanicalAssembly::getFirstComponentIntersectedByRay(const Ray& ray, P3D& worldP){
	double minDistance = DBL_MAX;
	KS_MechanicalComponent* closestComponent = NULL;
	for (uint i=0;i<m_components.size();i++){
		P3D p;
		if (m_components[i]->isIntersectedByRay(ray, p)){
			double dist = V3D(ray.origin, p).length();
			if (dist < minDistance){
				worldP = p;
				minDistance = dist;
				closestComponent = m_components[i];
			}
		}
	}

	return closestComponent;
}*/


void replaceSymbolsInString(const string& symbol, const string& val, const string& inputString, string& outputString){
	outputString.clear();
	outputString += inputString;

	//replace all the instances of the parameter names with their current values
	int pIndex = -1;
	while ((pIndex = (int)outputString.find(symbol)) != string::npos)
		outputString.replace(pIndex, symbol.size(), val);
}

#define ADD_SYMBOLS_IF_NEEDED(SYMB)																		\
		if (SYMB[0] != 'P'){																			\
			double v = atof(SYMB);																		\
			if (v != 0 && v != 1){																		\
				sprintf(pName, "P_%03d", pNames.size());												\
				pNames.push_back(string() + pName);														\
				pValues.push_back(SYMB);																\
				pIndex = -1;																			\
				replaceSymbolsInString(string() + SYMB, string() + pName, fileContents, tmpString);		\
				fileContents = tmpString;																\
				continue;																				\
			}																							\
		}																							


void KS_MechanicalAssembly::createParameterizedAssemblyFrom(const char* inputKSFile, const char* outputParameterizedAssemblyFile){
	string fileContents, tmpString;
	FILE* fp = fopen(inputKSFile, "r");

	DynamicArray<string> pNames;
	DynamicArray<string> pValues;

	//read the whole file into one large buffer...
	char buffer[1000];
	//this is where it happens.
	while (!feof(fp)){
		//get a line from the file...
		fgets(buffer, sizeof(buffer), fp);
		fileContents.append(buffer);
	}
	fclose(fp);

	int pIndex = -1;
	//now we need to look for all the pin connections and parameterize them...
	while ((pIndex = (int)fileContents.find(getKSString(KS_PIN_ON_COMP_IN), pIndex + 1)) != string::npos){
		//found a pin joint, now we need to see if we should create any symbols for it
		char symb1[100], symb2[100], pName[100];
		sscanf(fileContents.c_str() + pIndex + strlen(getKSString(KS_PIN_ON_COMP_IN)), "%s %s", symb1, symb2);
		//did we process these guys yet?
		ADD_SYMBOLS_IF_NEEDED(symb1);
		ADD_SYMBOLS_IF_NEEDED(symb2);
	}

	pIndex = -1;
	//now we need to look for all the pin connections and parameterize them...
	while ((pIndex = (int)fileContents.find(getKSString(KS_PIN_ON_COMP_OUT), pIndex + 1)) != string::npos){
		//found a pin joint, now we need to see if we should create any symbols for it
		char symb1[100], symb2[100], pName[100];
		sscanf(fileContents.c_str() + pIndex + strlen(getKSString(KS_PIN_ON_COMP_OUT)), "%s %s", symb1, symb2);
		//did we process these guys yet?
		ADD_SYMBOLS_IF_NEEDED(symb1);
		ADD_SYMBOLS_IF_NEEDED(symb2);
	}

	FILE* fOut = fopen(outputParameterizedAssemblyFile, "w");
	for (uint i=0;i<pNames.size();i++)
		fprintf(fOut, "%s %s %s\n", getKSString(KS_PARAMETER), pNames[i].c_str(), pValues[i].c_str());
	fprintf(fOut, "\n\n%s\n", fileContents.c_str());
	fclose(fp);

}

/*AABoundingBox KS_MechanicalAssembly::computeAABB(){
	AABoundingBox result;
	for (uint i=0;i<m_components.size();i++)
		result.combine(result, m_components[i]->computeAABB());
	return result;
}*/


void KS_MechanicalAssembly::writeMeshToFile(const char* objFName){
	FILE* f = fopen(objFName, "w");
	if (f == NULL){
		Logger::print("KS_MechanicalAssembly: Cannot load output file \'%s\'\n", objFName);
		return;
	}
	int vertexIdxOffset = 0;
	for(uint i=0;i<m_components.size();i++)
		vertexIdxOffset += m_components[i]->renderToObjFile(f, vertexIdxOffset);

	fclose(f);
}

void KS_MechanicalAssembly::solveAssembly()
{
	sSolver = s;

	if (1) {
		AConstraintEnergy->testGradientWithFD(sSolver);
		AConstraintEnergy->testHessianWithFD(sSolver);
	}

	double functionValue = AConstraintEnergy->computeValue(sSolver);
	Logger::consolePrint("energy value before solve C: %lf\n", functionValue);

	NewtonFunctionMinimizer minimizer(20);
	//BFGSFunctionMinimizer minimizer(10);
	minimizer.printOutput = false;
	minimizer.minimize(AConstraintEnergy, sSolver, functionValue);

	Logger::consolePrint("energy value after solve C: %lf\n", functionValue);
	

	s = sSolver;

}

