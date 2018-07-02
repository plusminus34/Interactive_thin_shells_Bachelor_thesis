#pragma once

#include <MathLib/Matrix.h>
#include "KS_Constraint.h"
#include "KS_MechanicalComponent.h"

/*
	ensures that the world coordinates of point x1 on component 1 are the same as that of point x2 on component 2. Alternatively, it ensures
	that vector x1 on component 1 is aligned to vecotr x2 on component 2. The difference is in the value of the flag that is provided as input.
*/

template <class T> class KS_V2VorP2PConstraint : public KS_Constraint{
public:
	KS_V2VorP2PConstraint(const T& x1_, KS_MechanicalComponent* c1_, const T& x2_, KS_MechanicalComponent* c2_, double w = 1.0){
		this->x1 = x1_;
		this->c1 = c1_;
		this->x2 = x2_;
		this->c2 = c2_;
		weight=w;
	}

	virtual KS_V2VorP2PConstraint* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut)const{
		KS_V2VorP2PConstraint* constraint = new KS_V2VorP2PConstraint(*this);
		constraint->c1=pCompIn;
		constraint->c2=pCompOut;
		return constraint;
	}

	virtual ~KS_V2VorP2PConstraint(void){
	}

	virtual void movePinOnC1(T x){this->x1 = x;}
	virtual void movePinOnC2(T x){this->x2 = x;}


	//compute the constraints and their jacobian, as well as the energy (1/2 C'C), and its gradient and hessian, all of them evaluated at the current state of the assembly
	virtual double getEnergy(){
		V3D v = getErrorVector();
		return 0.5 * v.dot(v);
	}

	virtual void computeEnergyGradient(){
		FAST_RESIZE_MAT(dw1_ds1, 3, KS_MechanicalComponent::getStateSize());
		FAST_RESIZE_MAT(dw2_ds2, 3, KS_MechanicalComponent::getStateSize());
		FAST_RESIZE_VEC(dE_ds1, KS_MechanicalComponent::getStateSize());
		FAST_RESIZE_VEC(dE_ds2, KS_MechanicalComponent::getStateSize());
		FAST_RESIZE_VEC(tmpV, 3);

		V3D v = getErrorVector();

		//the energy of the constraint is 1/2 * v' * v, and gradient follows

		c1->get_dw_ds(x1, dw1_ds1);
		dw1_ds1*=weight;
		c2->get_dw_ds(x2, dw2_ds2);
		dw2_ds2*=weight;

		tmpV[0] = v[0]; tmpV[1] = v[1]; tmpV[2] = v[2];

		//compute the gradient of the energy with respect to s1 and s2...
		preMultiply(dw1_ds1, tmpV, 1, dE_ds1);
		preMultiply(dw2_ds2, tmpV, -1, dE_ds2);
	}

	virtual void computeEnergyHessian(){
		FAST_RESIZE_MAT(dw1_ds1, 3, KS_MechanicalComponent::getStateSize());
		FAST_RESIZE_MAT(dw2_ds2, 3, KS_MechanicalComponent::getStateSize());
		FAST_RESIZE_MAT(tmpMat, 3, KS_MechanicalComponent::getStateSize());	
		FAST_RESIZE_MAT(ddE_ds1ds1, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
		FAST_RESIZE_MAT(ddE_ds1ds2, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
		FAST_RESIZE_MAT(ddE_ds2ds1, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
		FAST_RESIZE_MAT(ddE_ds2ds2, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());

		//the energy of the constraint is 1/2 * v' * v, and the hessian follows
		V3D v = getErrorVector();

		c1->get_dw_ds(x1, dw1_ds1);
		dw1_ds1*=weight;
		c2->get_dw_ds(x2, dw2_ds2);
		dw2_ds2*=weight;

		ddE_ds1ds1.setToATransposedB(dw1_ds1, dw1_ds1);
		ADD_v_TIMES_ddx_dsds_TO_HESSIAN(v, ddE_ds1ds1, x1, c1, 1);

		ddE_ds2ds2.setToATransposedB(dw2_ds2, dw2_ds2);
		ADD_v_TIMES_ddx_dsds_TO_HESSIAN(v, ddE_ds2ds2, x2, c2, -1);

		ddE_ds1ds2.setToATransposedB(dw1_ds1, dw2_ds2);
		ddE_ds1ds2 *= -1;

		ddE_ds2ds1.setToTransposeOf(ddE_ds1ds2);
	}

	virtual void computeConstraintJacobian(){
		FAST_RESIZE_MAT(dCds1, getConstraintCount(), KS_MechanicalComponent::getStateSize());
		FAST_RESIZE_MAT(dCds2, getConstraintCount(), KS_MechanicalComponent::getStateSize());

		c1->get_dw_ds(x1, dCds1);
		dCds1*=weight;
		c2->get_dw_ds(x2, dCds2);
		dCds2*=-weight;
	}

	virtual int getNumberOfAffectedComponents() {return 2;}
	virtual KS_MechanicalComponent* getIthAffectedComponent(int i){if (i==0) return c1; else return c2;}

	//for testing purposes only...
	virtual dVector* get_dE_dsi(int i) {if (i==0) return &dE_ds1; else return &dE_ds2;}
	virtual Matrix* get_ddE_dsidsj(int i, int j) {if (i == 0 && j == 0) return &ddE_ds1ds1; if (i==0 && j==1) return &ddE_ds1ds2; if (i==1 && j==0) return &ddE_ds2ds1; return &ddE_ds2ds2;}

	//each constraint is composed of several scalar constraints - this is how many
	virtual int getConstraintCount(){
		return 3;
	}
	//returns the current values of the constraints
	virtual dVector* getConstraintValues(){
		V3D v = getErrorVector();
		C.resize(3, 0);
		C[0] = v.x; C[1] = v.y; C[2] = v.z;
		return &C;
	}
	//returns the jacobian that tells us how the values of the constraint change with the state of the ith component involved in the constraint
	virtual Matrix* getConstraintJacobian(int i) {if (i == 0) return &dCds1; return &dCds2;}

private:
	V3D getErrorVector(){
		return (c1->get_w(x1) - c2->get_w(x2))*weight;
	}
	//we need to ensure that c1.W(x1) == c2.W(x2)
	T x1;
	KS_MechanicalComponent *c1;
	T x2;
	KS_MechanicalComponent *c2;

	//tmp variables
	Matrix dw1_ds1, dw2_ds2;
	dVector dE_ds1, dE_ds2;
	dVector tmpV;

	//these are the blocks for the energy hessian
	Matrix ddE_ds1ds1, ddE_ds1ds2, ddE_ds2ds2, ddE_ds2ds1, tmpMat;

	//and these are the blocks for the constraint jacobian
	Matrix dCds1, dCds2;

	//scalar constraints
	dVector C;

	double weight;
};


