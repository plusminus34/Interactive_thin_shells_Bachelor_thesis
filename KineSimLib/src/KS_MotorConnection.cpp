#include "KineSimLib/KS_MotorConnection.h"

KS_MotorConnection::KS_MotorConnection(void){
	ddAE_ds_dp1.resize(1, KS_MechanicalComponent::getStateSize()); ddAE_ds_dp1.setZero();
	ddAE_ds_dp2.resize(1, KS_MechanicalComponent::getStateSize()); ddAE_ds_dp2.setZero();
	m_offset = 0;
	isActuated = true;
}

KS_MotorConnection::~KS_MotorConnection(void){
}





