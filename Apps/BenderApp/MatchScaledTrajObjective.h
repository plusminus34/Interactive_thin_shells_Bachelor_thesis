#pragma once

#include "MathLib/P3D.h"


#include "MeshObjective.h"
#include "Trajectory3Dplus.h"

class MatchScaledTrajObjective : public MeshObjective
{
public:


	DynamicArray<Node * > matchedFiber;
	Trajectory3Dplus targetTrajectory;

	// helpers
	DynamicArray<double> tNodeFiber;
	DynamicArray<double> tNodeTarget;

	DynamicArray<DynamicArray<V3D> > dtTargetDxNode;

	//DynamicArray<V3D> dtTargeti_DxNodej_jsmalleri; // for each node j: the influence this node j has on the t of ANY node i
	//DynamicArray<V3D> dtTargeti_DxNodej_jbiggeri;
	//DynamicArray<V3D> dtTargeti_DxNodej_jequalsi;

public:
	MatchScaledTrajObjective(DynamicArray<Node * > matchedFiber, Trajectory3Dplus targetTrajectory);

	virtual void addO(const dVector & x, const dVector & X, double & o);
	virtual void addDoDx(const dVector & x, const dVector & X, dVector & dodx);

	virtual void addError(const dVector & x, double & e);

	//void draw(dVector const & x);


	// helpers
	void update_tNode(dVector const & x);
	double computeOofNode(int nodeID_local, const dVector & x);
	void addDoDxEachNode(int nodeID_local, const dVector & x, dVector & dodx);

	void prepareDtTargetDxNode(dVector const & x);

	void draw(dVector const & x);

};



