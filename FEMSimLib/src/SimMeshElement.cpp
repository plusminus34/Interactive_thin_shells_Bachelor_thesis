#include <FEMSimLib/SimMeshElement.h>
#include <FEMSimLib/SimulationMesh.h>

SimMeshElement::SimMeshElement(SimulationMesh* simMesh){
	this->simMesh = simMesh;
}

SimMeshElement::~SimMeshElement(){

}
