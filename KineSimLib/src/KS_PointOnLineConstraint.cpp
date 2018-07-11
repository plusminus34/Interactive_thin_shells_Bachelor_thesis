#include "KineSimLib/KS_PointOnLineConstraint.h"


KS_PointOnLineConstraint::KS_PointOnLineConstraint(const P3D& xOnC1_, KS_MechanicalComponent* c1_, const P3D& pOnC2_, const V3D& lOnC2_, KS_MechanicalComponent* c2_){
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
	V3D v = getErrorVector();
	return 0.5 * v.dot(v);
}

V3D KS_PointOnLineConstraint::getErrorVector(){
	wx = c1->get_w(x);
	wp = c2->get_w(p);
	wl = c2->get_w(l);

	//wl cross (x-p) is zero when the vector (x-p) is aligned with l (i.e. x is on the line), so we'll use this as a measure
	//of the error...
	return wl.cross(wx-wp);
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

	V3D y = getErrorVector();
	V3D v = (wx - wp);

	//the energy of the constraint is 1/2 * y' * y, and gradient and hessian follow
	c1->get_dw_ds(x, dwx_ds1);
	c2->get_dw_ds(p, dwp_ds2);
	c2->get_dw_ds(l, dwl_ds2);

	//toCrossProductMatrix(wlCross, wl);
	wlCross = getCrossProductMatrix(wl);
   //toCrossProductMatrix(wvCross, v);
	wvCross = getCrossProductMatrix(v);
	tmpY[0] = y[0]; tmpY[1] = y[1]; tmpY[2] = y[2];

	//compute dE_ds1 = y' * lx * dx/ds1 (lx = cross product matrix corresponding to l)
	//dy_ds1.setToAB(wlCross, dwx_ds1);
	dy_ds1 = wlCross * dwx_ds1;

	//preMultiply(dy_ds1, tmpY, 1, dE_ds1);
	dE_ds1 = tmpY.transpose()*dy_ds1;
	//compute dE_ds2 = -y' * lx * dp/ds2 -v'* vx * (lx = cross product matrix corresponding to l, vx same, but for v)
	//dy_ds2.setToAB(wlCross, dwp_ds2);
	dy_ds2 = wlCross * dwp_ds2;
	//dy_ds2.addProductAB(wvCross, dwl_ds2);
	dy_ds2 += wvCross * dwl_ds2;
	//preMultiply(dy_ds2, tmpY, -1, dE_ds2);
	dE_ds2 = -1 * tmpY.transpose() * dy_ds2; // double check
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

	V3D y = getErrorVector();
	V3D v = (wx - wp);

	//the energy of the constraint is 1/2 * y' * y, and gradient and hessian follow
	c1->get_dw_ds(x, dwx_ds1);
	c2->get_dw_ds(p, dwp_ds2);
	c2->get_dw_ds(l, dwl_ds2);

	//toCrossProductMatrix(wlCross, wl);
	wlCross=getCrossProductMatrix(wl),
	//toCrossProductMatrix(wvCross, v);
	wvCross = getCrossProductMatrix(v);
	tmpY[0] = y[0]; tmpY[1] = y[1]; tmpY[2] = y[2];

	//compute dE_ds1 = y' * lx * dx/ds1 (lx = cross product matrix corresponding to l)
	//dy_ds1.setToAB(wlCross, dwx_ds1);
	dy_ds1=wlCross*dwx_ds1;
	

	//compute dE_ds2 = -y' * lx * dp/ds2 -v'* vx * (lx = cross product matrix corresponding to l, vx same, but for v)
	//dy_ds2.setToAB(wlCross, dwp_ds2);
	dy_ds2=wlCross*dwp_ds2;
	//dy_ds2.addProductAB(wvCross, dwl_ds2);
	dy_ds2 += wvCross * dwl_ds2;
	dy_ds2 *= -1;

	//now ddE_ds1ds1
	//ddE_ds1ds1.setToATransposedB(dy_ds1, dy_ds1);
	ddE_ds1ds1=dy_ds1.transpose()*dy_ds1;
	//below is the tensor product between y'*lx and ddx/ds1ds1
	//preMultiply(wlCross, tmpY, 1, tmpV);
	tmpV = tmpY.transpose()*wlCross;
	ADD_v_TIMES_ddx_dsds_TO_HESSIAN(tmpV, ddE_ds1ds1, x, c1, 1);

	//and ddE_ds2ds2
	//ddE_ds2ds2.setToATransposedB(dy_ds2, dy_ds2);
	ddE_ds2ds2 = dy_ds2.transpose()*dy_ds2;
	//preMultiply(wlCross, tmpY, -1, tmpV);
	tmpV = -1 * tmpY.transpose()*wlCross;
	ADD_v_TIMES_ddx_dsds_TO_HESSIAN(tmpV, ddE_ds2ds2, p, c2, 1);
	//preMultiply(wvCross, tmpY, -1, tmpV);
	tmpV = -1 * tmpY.transpose()*wvCross;
	ADD_v_TIMES_ddx_dsds_TO_HESSIAN(tmpV, ddE_ds2ds2, l, c2, 1);

	V3D col_i;
	for (int i=0;i<KS_MechanicalComponent::getStateSize();i++){
		col_i[0] = dwp_ds2(0, i); col_i[1] = dwp_ds2(1, i); col_i[2] = dwp_ds2(2, i);
		//toCrossProductMatrix(tmpCross, col_i);
		tmpCross = getCrossProductMatrix(col_i);
		//preMultiply(tmpCross, tmpY, 1, tmpV);
		tmpV = tmpY.transpose()*tmpCross;
		//preMultiply(dwl_ds2, tmpV, 1, tmpE);
		tmpE = tmpV.transpose()*dwl_ds2;
		for (int j=0;j<KS_MechanicalComponent::getStateSize();j++)
			ddE_ds2ds2(i,j) += tmpE[j];

		col_i[0] = dwl_ds2(0, i); col_i[1] = dwl_ds2(1, i); col_i[2] = dwl_ds2(2, i);
		//toCrossProductMatrix(tmpCross, col_i);
		tmpCross = getCrossProductMatrix(col_i);
		//preMultiply(tmpCross, tmpY, -1, tmpV);
		tmpV = -1 * tmpY.transpose()*tmpCross;
		//preMultiply(dwp_ds2, tmpV, 1, tmpE);
		tmpE = tmpV.transpose()*dwp_ds2;
		for (int j=0;j<KS_MechanicalComponent::getStateSize();j++)
			ddE_ds2ds2(i,j) += tmpE[j];
	}

	//finally the mixed derivatives - start with ddE_ds1ds2
	//ddE_ds2ds1.setToATransposedB(dy_ds2, dy_ds1);
	ddE_ds2ds1=dy_ds2.transpose()*dy_ds1;
	for (int i=0;i<KS_MechanicalComponent::getStateSize();i++){
		col_i[0] = dwx_ds1(0, i); col_i[1] = dwx_ds1(1, i); col_i[2] = dwx_ds1(2, i);
		//toCrossProductMatrix(tmpCross, col_i);
		tmpCross = getCrossProductMatrix(col_i);
		//preMultiply(tmpCross, tmpY, -1, tmpV);
		tmpV = -1 * tmpY.transpose()*tmpCross;
		//preMultiply(dwl_ds2, tmpV, 1, tmpE);
		tmpE = tmpV.transpose()*dwl_ds2;
		for (int j=0;j<KS_MechanicalComponent::getStateSize();j++)
			ddE_ds2ds1(j,i) += tmpE[j];
	}

	//ddE_ds1ds2.setToTransposeOf(ddE_ds2ds1);
	ddE_ds1ds2=ddE_ds2ds1.transpose();
}

int KS_PointOnLineConstraint::getConstraintCount() {
	return 3;
}
