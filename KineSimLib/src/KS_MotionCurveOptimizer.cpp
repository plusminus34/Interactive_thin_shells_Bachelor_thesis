#include "KineSimLib/KS_MotionCurveOptimizer.h"

KS_MotionCurveOptimizer::KS_MotionCurveOptimizer(KS_ParameterizedMechanicalAssembly* mechanism, int numberOfEvaluationPoints, int targetComponentIndex, Point3d localCoordsOfTracerPoint) : KS_ParameterizedMechanismOptimizerBase(mechanism, numberOfEvaluationPoints){
	this->targetComponentIndex = targetComponentIndex;
	this->localCoordsOfTracerPoint = localCoordsOfTracerPoint;
	setCurrentBestSolution(*mechanism->getParameterVector());
	curveMatchOffset = 0;
	regularizer = 0.1;
	p0 = bestParams;
}

KS_MotionCurveOptimizer::~KS_MotionCurveOptimizer(){
}

void KS_MotionCurveOptimizer::setTargetMotionCurve(const PolyLine3d& target){
	motionTrailTarget = target;
	this->computeAssemblyMotionGivenParameters(bestParams, &tmpAssemblyStateArray, &tmpTickerValues);
	curveMatchOffset = this->computeBestCurveMatchOffset(&tmpAssemblyStateArray);
}

Point3d KS_MotionCurveOptimizer::getTargetPointForFrame(int i){
	return getTargetPointForFrame(i, curveMatchOffset);
}

Point3d KS_MotionCurveOptimizer::getTargetPointForFrame(int i, double offset){
	double t = (double)(i % numberOfEvaluationPoints)/numberOfEvaluationPoints + offset;
	if (t > 1) t -= 1;
	return motionTrailTarget.points.evaluate_linear(t);
}


double KS_MotionCurveOptimizer::getDistanceToTargetCurve(DynamicArray<dVector> *stateArray, double offset){
	assert(offset >= 0 && offset <=1);
	//we compute the distance at the point sampled on the tracer particle
	double distance = 0;
	int nPoints = (int)stateArray->size();
	for (int i=0;i<nPoints;i++){
		parameterizedAssembly->getCurrentMechanicalAssembly()->setAssemblyState(stateArray->at(i));
		distance += Vector3d(parameterizedAssembly->getCurrentMechanicalAssembly()->getComponent(targetComponentIndex)->get_w(localCoordsOfTracerPoint), getTargetPointForFrame(i, offset)).length2();
	}
	parameterizedAssembly->getCurrentMechanicalAssembly()->setAssemblyState(startState);
	return sqrt(distance);
}

double KS_MotionCurveOptimizer::computeBestCurveMatchOffset(DynamicArray<dVector> *stateArray){
	double bestOffset = 0;
	double bestDistance = DBL_MAX;
	//do a brute force comparison of the target and actual motion curves and figure out roughly what the best offset is...
	int nPoints = (int)stateArray->size();
	for (int i=0;i<nPoints;i++){
		double distance = getDistanceToTargetCurve(stateArray, (double)i/nPoints);
//		logPrint("%lf\t", distance);
		if (distance < bestDistance){
			bestDistance = distance;
			bestOffset = (double)i/nPoints;
		}
	}
//	logPrint("\n");

	return bestOffset;
}

//this should always return the current value of the objective function
double KS_MotionCurveOptimizer::computeValue(const dVector& p){
	bool allConverged = this->computeAssemblyMotionGivenParameters(p, &tmpAssemblyStateArray, &tmpTickerValues);

	//TODO: might want to make sure the assembly did not flip in the process...
	if (!allConverged)
		return DBL_MAX;	

	double val = getDistanceToTargetCurve(&tmpAssemblyStateArray, curveMatchOffset);

	//add the regularizer contribution
	dVector vd((int)p.size(), 0);
	add(p,1.0,p0,-1.0,vd);
	double nrmvd2 = dotprod(vd,vd);

	return 0.5 * val * val + 0.5 * regularizer * nrmvd2;
}

//this method gets called whenever a new best solution to the objective function is found
void KS_MotionCurveOptimizer::setCurrentBestSolution(const dVector& p){
	bestParams = p;
	computeAssemblyMotionGivenParameters(p, &tmpAssemblyStateArray, &tmpTickerValues);
	updateStartingState();
}

//compute gradient of the objective function...
dVector* KS_MotionCurveOptimizer::getGradientAt(const dVector& params){
	dVector tmpParams = params;
	this->computeAssemblyMotionGivenParameters(params, &tmpAssemblyStateArray, &tmpTickerValues);
	parameterizedAssembly->compute_dsdp(&tmpAssemblyStateArray, &tmpTickerValues, &tmpParams, &dsdpArray);

	//Gradient computation:

	//f = 1/2 * sum (xi-xT)'(xi-xT),
	//so df/dp = sum (dxi/dp)'(xi-xT), where dxi/dp = dxi/dsi * dsi/dp. dsi/dp is already being computed, so we need (xi-xT) and dxi/dsi
	//where df/dp is px1 (px3x3x1), dxi/dp is 3xp, dx/ds is 3xs, ds/dp is sxp.

	assert(parameterizedAssembly->getCurrentMechanicalAssembly()->getComponent(targetComponentIndex)->getStateSize() == 6);
	int p = parameterizedAssembly->getParameterCount();
	int s = parameterizedAssembly->getCurrentMechanicalAssembly()->getComponentCount() * 6;
	Matrix dxds(3, 6);
	Matrix dxdp(3, p);
	dVector xErr(3, 0);
	dVector dfdpi(p, 0);
	gradient.resize(p);
	gradient.zero();
	int nPoses = (int)tmpAssemblyStateArray.size();

	for (int i=0; i < nPoses; i++){
		dxdp.setToZeros();
		parameterizedAssembly->getCurrentMechanicalAssembly()->setAssemblyState(tmpAssemblyStateArray.at(i));
		//this is the (xi-xT) term...
		Vector3d vError(getTargetPointForFrame(i), parameterizedAssembly->getCurrentMechanicalAssembly()->getComponent(targetComponentIndex)->get_w(localCoordsOfTracerPoint));
		//this is part of the dxi/dsi term. The rest of it is zero, because the position of the marker point only depends on one component, so careful how we multiply this by dsi/dp
		parameterizedAssembly->getCurrentMechanicalAssembly()->getComponent(targetComponentIndex)->get_dw_ds(localCoordsOfTracerPoint, dxds);

		int nonZeroStateStartIndex = parameterizedAssembly->getCurrentMechanicalAssembly()->getComponent(targetComponentIndex)->getComponentIndex() * 6;

		for (int j=0;j<3;j++)
			for (int k=0;k<p;k++)
				for (int l=0;l<6;l++)
					dxdp(j, k) += dxds(j, l) * dsdpArray[i][k * s + nonZeroStateStartIndex + l];

		xErr[0] = vError.x; xErr[1] = vError.y; xErr[2] = vError.z;

		//now multiply dx/dp by xError to get df/dp.
		dxdp.multiplyTransposed(xErr, dfdpi);
		add(gradient, 1.0, dfdpi, 1.0, gradient);

	}

	//add the regularizer contribution
	dVector vd((int)params.size(), 0);
	add(params,1.0,p0,-1.0,vd);
	add(gradient, 1.0, vd, regularizer, gradient);

	//and done...
	return &gradient;
}

