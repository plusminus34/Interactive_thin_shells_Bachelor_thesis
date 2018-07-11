#pragma once


#include <Eigen/Sparse>
#include "KS_MechanicalComponent.h"

#define ADD_v_TIMES_ddx_dsds_TO_HESSIAN(v, hessian, x, c, scale){									\
		c->get_ddw_dgds(x, tmpMat);																	\
		for (int iii=0;iii<KS_MechanicalComponent::getStateSize();iii++)											\
			hessian(0,iii) += scale*(tmpMat(0,iii) * v[0] + tmpMat(1,iii) * v[1] + tmpMat(2,iii) * v[2]);		\
		c->get_ddw_dbds(x, tmpMat);																	\
		for (int iii=0;iii<KS_MechanicalComponent::getStateSize();iii++)											\
			hessian(1,iii) += scale*(tmpMat(0,iii) * v[0] + tmpMat(1,iii) * v[1] + tmpMat(2,iii) * v[2]);		\
		c->get_ddw_dads(x, tmpMat);																	\
		for (int iii=0;iii<KS_MechanicalComponent::getStateSize();iii++)											\
			hessian(2,iii) += scale*(tmpMat(0,iii) * v[0] + tmpMat(1,iii) * v[1] + tmpMat(2,iii) * v[2]);		\
	}

#define FAST_RESIZE_MAT(mat, m, n) {if (mat.rows()!=m || mat.cols()!=n) mat.resize(m,n);mat.setZero();}
#define FAST_RESIZE_VEC(vec, m) {if (vec.size()!=m) vec.resize(m); vec.setZero();}


/**
	interface for generic constraints applied to the state of mechanical components. For every constraint we have an energy term
	that tells us how well the constraint is satisfied (when this is 0, it means it's fully satisfied), and we also have information
	regarding the gradient and hessian of this energy term.
*/

class KS_Constraint{
public:
	KS_Constraint(void);
	virtual KS_Constraint* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut) const = 0;
	virtual ~KS_Constraint(void);

	void setConstraintStartIndex(int index) {constraintStartIndex = index;}
	int getConstraintStartIndex(){return constraintStartIndex;} 

	//add energy gradient and hessian of this constraint to the global ones...
	virtual void addEnergyGradientTo(dVector& gradient);
	//virtual void addEnergyHessianTo(SparseMatrix& hessian);
	virtual void addEnergyHessianTo(DynamicArray<MTriplet>& hessianEntries);


	//compute the constraints and their jacobian, as well as the energy (1/2 C'C), and its gradient and hessian, all of them evaluated at the current state of the assembly
	virtual double getEnergy() = 0;
	virtual void computeEnergyGradient() = 0;
	virtual void computeEnergyHessian() = 0;

	virtual int getNumberOfAffectedComponents() = 0;
	virtual KS_MechanicalComponent* getIthAffectedComponent(int i) = 0;

	virtual dVector* get_dE_dsi(int i) = 0;

	virtual MatrixNxM* get_ddE_dsidsj(int i, int j) = 0;

	void setAffectedComponentsState(const dVector& state);
protected:
	/*std::vector<SparseMatrixBlockHandle> hBlocks;
	std::vector<SparseMatrixBlockHandle> dCdsBlocks;
	std::vector<SparseMatrixBlockHandle> dCdsiBlocks;*/

	//this will keep track of 
	int constraintStartIndex;
};

