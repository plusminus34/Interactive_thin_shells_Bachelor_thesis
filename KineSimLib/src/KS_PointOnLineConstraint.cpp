
#include "KineSimLib/KS_PointOnLineConstraint.h"


KS_PointOnLineConstraint::KS_PointOnLineConstraint(const Point3d& xOnC1_, KS_MechanicalComponent* c1_, const Point3d& pOnC2_, const Vector3d& lOnC2_, KS_MechanicalComponent* c2_){
	x = xOnC1_;
	c1 = c1_;
	p = pOnC2_;
	l = lOnC2_;
	c2 = c2_;
}

KS_PointOnLineConstraint* KS_PointOnLineConstraint::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut)const{
	KS_PointOnLineConstraint* constraint = new KS_PointOnLineConstraint(*this);
	constraint->c1=pCompIn;
	constraint->c2=pCompOut;
	return constraint;
}

KS_PointOnLineConstraint::~KS_PointOnLineConstraint(void){
}

double KS_PointOnLineConstraint::getEnergy(){
	Vector3d v = getErrorVector();
	return 0.5 * v.dotProductWith(v);
}

Vector3d KS_PointOnLineConstraint::getErrorVector(){
	wx = c1->get_w(x);
	wp = c2->get_w(p);
	wl = c2->get_w(l);

	//wl cross (x-p) is zero when the vector (x-p) is aligned with l (i.e. x is on the line), so we'll use this as a measure
	//of the error...
	return wl.cross(wx-wp);
}

//each constraint is composed of several scalar constraints - this is how many
int KS_PointOnLineConstraint::getConstraintCount(){
	return 3;
}

//returns the current values of the constraints
dVector* KS_PointOnLineConstraint::getConstraintValues(){
	Vector3d v = getErrorVector();
	C.resize(3, 0);
	C[0] = v.x; C[1] = v.y; C[2] = v.z;
	return &C;
}

void KS_PointOnLineConstraint::computeConstraintJacobian(){
	FAST_RESIZE_MAT(dwx_ds1, 3, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(dwp_ds2, 3, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(dwl_ds2, 3, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(wlCross, 3, 3);
	FAST_RESIZE_MAT(wvCross, 3, 3);

	FAST_RESIZE_MAT(dCds1, getConstraintCount(), KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(dCds2, getConstraintCount(), KS_MechanicalComponent::getStateSize());

	Vector3d v = (wx - wp);
	//the energy of the constraint is 1/2 * y' * y, and gradient and hessian follow
	c1->get_dw_ds(x, dwx_ds1);
	c2->get_dw_ds(p, dwp_ds2);
	c2->get_dw_ds(l, dwl_ds2);

	toCrossProductMatrix(wlCross, wl);
	toCrossProductMatrix(wvCross, v);

	//compute dC_ds1 = lx * dx/ds1 (lx = cross product matrix corresponding to l)
	dCds1.setToAB(wlCross, dwx_ds1);

	//compute dE_ds2 = -lx * dp/ds2 -v'* vx * (lx = cross product matrix corresponding to l, vx same, but for v)
	dCds2.setToAB(wlCross, dwp_ds2);
	dCds2.addProductAB(wvCross, dwl_ds2);
	dCds2 *= -1;
}

void KS_PointOnLineConstraint::computeEnergyGradient(){
	FAST_RESIZE_MAT(dwx_ds1, 3, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(dwp_ds2, 3, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(dwl_ds2, 3, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(dy_ds1, 3, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(dy_ds2, 3, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(wlCross, 3, 3);
	FAST_RESIZE_MAT(wvCross, 3, 3);

	FAST_RESIZE_VEC(dE_ds1, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_VEC(dE_ds2, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_VEC(tmpE, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_VEC(tmpY, 3);

	Vector3d y = getErrorVector();
	Vector3d v = (wx - wp);

	//the energy of the constraint is 1/2 * y' * y, and gradient and hessian follow
	c1->get_dw_ds(x, dwx_ds1);
	c2->get_dw_ds(p, dwp_ds2);
	c2->get_dw_ds(l, dwl_ds2);

	toCrossProductMatrix(wlCross, wl);
	toCrossProductMatrix(wvCross, v);
	tmpY[0] = y[0]; tmpY[1] = y[1]; tmpY[2] = y[2];

	//compute dE_ds1 = y' * lx * dx/ds1 (lx = cross product matrix corresponding to l)
	dy_ds1.setToAB(wlCross, dwx_ds1);
	dCds1 = dy_ds1;
	preMultiply(dy_ds1, tmpY, 1, dE_ds1);
	
	//compute dE_ds2 = -y' * lx * dp/ds2 -v'* vx * (lx = cross product matrix corresponding to l, vx same, but for v)
	dy_ds2.setToAB(wlCross, dwp_ds2);
	dy_ds2.addProductAB(wvCross, dwl_ds2);
	preMultiply(dy_ds2, tmpY, -1, dE_ds2);
}

void KS_PointOnLineConstraint::computeEnergyHessian(){
	FAST_RESIZE_MAT(dwx_ds1, 3, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(dwp_ds2, 3, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(dwl_ds2, 3, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(dy_ds1, 3, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(dy_ds2, 3, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(tmpMat, 3, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(wlCross, 3, 3);
	FAST_RESIZE_MAT(wvCross, 3, 3);
	FAST_RESIZE_MAT(tmpCross, 3, 3);	

	FAST_RESIZE_MAT(ddE_ds1ds1, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(ddE_ds1ds2, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(ddE_ds2ds1, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(ddE_ds2ds2, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_VEC(tmpE, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_VEC(tmpY, 3);
	FAST_RESIZE_VEC(tmpV, 3);

	FAST_RESIZE_MAT(dCds1, getConstraintCount(), KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(dCds2, getConstraintCount(), KS_MechanicalComponent::getStateSize());

	Vector3d y = getErrorVector();
	Vector3d v = (wx - wp);

	//the energy of the constraint is 1/2 * y' * y, and gradient and hessian follow
	c1->get_dw_ds(x, dwx_ds1);
	c2->get_dw_ds(p, dwp_ds2);
	c2->get_dw_ds(l, dwl_ds2);

	toCrossProductMatrix(wlCross, wl);
	toCrossProductMatrix(wvCross, v);
	tmpY[0] = y[0]; tmpY[1] = y[1]; tmpY[2] = y[2];

	//compute dE_ds1 = y' * lx * dx/ds1 (lx = cross product matrix corresponding to l)
	dy_ds1.setToAB(wlCross, dwx_ds1);
	

	//compute dE_ds2 = -y' * lx * dp/ds2 -v'* vx * (lx = cross product matrix corresponding to l, vx same, but for v)
	dy_ds2.setToAB(wlCross, dwp_ds2);
	dy_ds2.addProductAB(wvCross, dwl_ds2);
	dy_ds2 *= -1;

	//now ddE_ds1ds1
	ddE_ds1ds1.setToATransposedB(dy_ds1, dy_ds1);
	//below is the tensor product between y'*lx and ddx/ds1ds1
	preMultiply(wlCross, tmpY, 1, tmpV);
	ADD_v_TIMES_ddx_dsds_TO_HESSIAN(tmpV, ddE_ds1ds1, x, c1, 1);

	//and ddE_ds2ds2
	ddE_ds2ds2.setToATransposedB(dy_ds2, dy_ds2);
	preMultiply(wlCross, tmpY, -1, tmpV);
	ADD_v_TIMES_ddx_dsds_TO_HESSIAN(tmpV, ddE_ds2ds2, p, c2, 1);
	preMultiply(wvCross, tmpY, -1, tmpV);
	ADD_v_TIMES_ddx_dsds_TO_HESSIAN(tmpV, ddE_ds2ds2, l, c2, 1);

	Vector3d col_i;
	for (int i=0;i<KS_MechanicalComponent::getStateSize();i++){
		col_i[0] = dwp_ds2(0, i); col_i[1] = dwp_ds2(1, i); col_i[2] = dwp_ds2(2, i);
		toCrossProductMatrix(tmpCross, col_i);
		preMultiply(tmpCross, tmpY, 1, tmpV);
		preMultiply(dwl_ds2, tmpV, 1, tmpE);
		for (int j=0;j<KS_MechanicalComponent::getStateSize();j++)
			ddE_ds2ds2(i,j) += tmpE[j];

		col_i[0] = dwl_ds2(0, i); col_i[1] = dwl_ds2(1, i); col_i[2] = dwl_ds2(2, i);
		toCrossProductMatrix(tmpCross, col_i);
		preMultiply(tmpCross, tmpY, -1, tmpV);
		preMultiply(dwp_ds2, tmpV, 1, tmpE);
		for (int j=0;j<KS_MechanicalComponent::getStateSize();j++)
			ddE_ds2ds2(i,j) += tmpE[j];
	}

	//finally the mixed derivatives - start with ddE_ds1ds2
	ddE_ds2ds1.setToATransposedB(dy_ds2, dy_ds1);
	for (int i=0;i<KS_MechanicalComponent::getStateSize();i++){
		col_i[0] = dwx_ds1(0, i); col_i[1] = dwx_ds1(1, i); col_i[2] = dwx_ds1(2, i);
		toCrossProductMatrix(tmpCross, col_i);
		preMultiply(tmpCross, tmpY, -1, tmpV);
		preMultiply(dwl_ds2, tmpV, 1, tmpE);
		for (int j=0;j<KS_MechanicalComponent::getStateSize();j++)
			ddE_ds2ds1(j,i) += tmpE[j];
	}

	ddE_ds1ds2.setToTransposeOf(ddE_ds2ds1);
}

