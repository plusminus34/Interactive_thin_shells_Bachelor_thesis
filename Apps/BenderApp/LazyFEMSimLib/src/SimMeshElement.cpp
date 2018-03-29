#include <LazyFEMSimLib/SimMeshElement.h>
#include <LazyFEMSimLib/SimulationMesh.h>

SimMeshElement::SimMeshElement(SimulationMesh* simMesh){
	this->simMesh = simMesh;
}

SimMeshElement::~SimMeshElement(){

}
