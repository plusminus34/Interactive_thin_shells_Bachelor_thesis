#include "KineSimLib/KS_MechanicalAssembly.h"
#include "KineSimLib/KS_LoaderUtils.h"
#include "KineSimLib/KS_BindComponentsConnection.h"
#include "KineSimLib/KS_BoundToWorldConnection.h"
#include "KineSimLib/KS_PointOnLineConnection.h"
#include "KineSimLib/KS_rMotorConnection.h"
#include "KineSimLib/KS_GenericComponent.h"
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
	AConstraintEnergy = NULL;
}

KS_MechanicalAssembly::KS_MechanicalAssembly(KS_MechanicalAssembly &a){
	m_components.resize(a.getComponents().size());
	m_connections.resize(a.getConnections().size());
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
		m_connections[i] = a.getConnection(i)->clone(mCompIn,mCompOut);
	}
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
			case KS_R_MOTOR_CON:{
					KS_rMotorConnection* mC = new KS_rMotorConnection();
					mC->loadFromFile(f, this);
					this->addConnection(mC);
				}break;
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
	for (uint i = 0; i < m_components.size(); i++) {
		m_components[i]->setupGeometry();
		s[KS_MechanicalComponent::getStateSize() * i + 0]= m_components[i]->getGamma(); 
	    s[KS_MechanicalComponent::getStateSize() * i + 1]= m_components[i]->getBeta(); 
		s[KS_MechanicalComponent::getStateSize() * i + 2]= m_components[i]->getAlpha(); 
		s[KS_MechanicalComponent::getStateSize() * i + 3]= m_components[i]->getWorldCenterPosition()[0];
		s[KS_MechanicalComponent::getStateSize() * i + 4]= m_components[i]->getWorldCenterPosition()[1];
	    s[KS_MechanicalComponent::getStateSize() * i + 5]= m_components[i]->getWorldCenterPosition()[2];
	}
	setActuatedConnections();
	AConstraintEnergy = new KS_AssemblyConstraintEnergy();
	AConstraintEnergy->initialize(this);
	return true;
}

void KS_MechanicalAssembly::readMechStateFromFile(const char * fName)
{
	FILE* fp = fopen(fName, "r");
	double temp;
	for (int i = 0; i < s.size(); i++) {
		fscanf(fp, "%lf", &temp);
		s[i] = temp;
	}
	fclose(fp);
}

void KS_MechanicalAssembly::writeToFile(const char* szFile){
	FILE* f = fopen(szFile, "w");
	if (f == NULL){
		Logger::print("KS_MechanicalAssembly: Cannot open input file \'%s\' for writing\n", szFile);
		return;
	}

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
	m_connections.push_back(pConn);
}


void KS_MechanicalAssembly::stepAssembly(){
	for (uint i=0;i<m_connections.size();i++)
		m_connections[i]->updateConnection();
}


void KS_MechanicalAssembly::draw(){
	for (uint i=0;i<this->m_components.size();i++){
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

	if (newtonSolver) {
		NewtonFunctionMinimizer minimizer(20);
		minimizer.printOutput = false;
		minimizer.minimize(AConstraintEnergy, sSolver, functionValue);
	}
		
	if (bfgsSolver) {
		BFGSFunctionMinimizer minimizer(10);
		minimizer.printOutput = false;
		minimizer.minimize(AConstraintEnergy, sSolver, functionValue);
	}
	

	Logger::consolePrint("energy value after solve C: %lf\n", functionValue);
	

	s = sSolver;

}

void KS_MechanicalAssembly::logMechS(const char* szFile)
{
	FILE* fp2 = fopen(szFile,"w");
	for (int i = 0; i < s.size(); i++) {
		fprintf(fp2, "%lf\n", s[i]);
	}
	/*(fp2, "%lf\n", xEE);
	fprintf(fp2, "%lf\n", yEE);
	fprintf(fp2, "%lf\n", thEE);*/

	/*for (int i = 0; i<s.size(); i++) {
		fprintf(fp2, "%lf\n", p[i]);
	}*/
	fclose(fp2);
}

void KS_MechanicalAssembly::setActuatedConnections()
{
	actuated_connections.clear();
	for (uint i = 0; i < m_connections.size(); i++) {
		if (m_connections[i]->isMotorized())
			actuated_connections.push_back(m_connections[i]);
	}
}

void KS_MechanicalAssembly::updateActuatedConnections()
{
	for (uint i = 0; i < actuated_connections.size(); i++) {
			actuated_connections[i]->updateConnection();
	}
}

