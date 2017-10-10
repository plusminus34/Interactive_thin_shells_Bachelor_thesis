#include <OptimizationLib/CMAIterativeFunctionMinimizer.h>


CMAIterativeFunctionMinimizer::CMAIterativeFunctionMinimizer(int populationSize, double solveFunctionValue, double solveHistToleranceValue)
	:	function(NULL),
		populationSize(populationSize),
		solveFunctionValue(solveFunctionValue),
		solveHistToleranceValue(solveHistToleranceValue),
		initialStdDevValue(0.01),
		printLevel(0),
		initialized(false),
		arFunVals(NULL),
		totalNumberOfIterations(0)
{
}

CMAIterativeFunctionMinimizer::~CMAIterativeFunctionMinimizer()
{
	if (arFunVals != NULL)
		cmaes_exit(&evo);
}

void CMAIterativeFunctionMinimizer::setObjective(ObjectiveFunction *function)
{
	this->function = function;
	initialized = false;
}

void CMAIterativeFunctionMinimizer::setInitialStandardDeviation(double initialStdDev)
{
	initialStdDevValue = initialStdDev;
	initialized = false;
}

void CMAIterativeFunctionMinimizer::setInitialStandardDeviation(const dVector &initialStdDev, double stdDevModifier)
{
	this->initialStdDev = initialStdDev * stdDevModifier;
	this->initialStdDevValue = 0.01;
	initialized = false;
}

void CMAIterativeFunctionMinimizer::setInitialParameterSet(const dVector &p0, const dVector &pMin, const dVector &pMax)
{
	this->p = p0;
	this->pMin = pMin;
	this->pMax = pMax;
	initialized = false;
}

void CMAIterativeFunctionMinimizer::initialize()
{
	if (function == NULL)
		throwError("CMA: Function is not set");
	if (p.size()==0 || pMin.size() != p.size() || pMax.size() != p.size())
		throwError("CMA: Initial parameters not set correctly");

	if (initialStdDev.size() != p.size())
	{
		initialStdDev.resize(p.size());
		for (int i=0; i<p.size(); i++)
			initialStdDev[i] = initialStdDevValue;
	}

	if (arFunVals != NULL)
	{
		cmaes_exit(&evo);
		arFunVals = NULL;
	}
	
	double initialFunctionValue = function->computeValue(p);
	
	if (printLevel >= 1) {
		Logger::logPrint("\n=========== Starting CMA minimization ===========\n");
		Logger::logPrint("   Initial function value: %.5lf\n", initialFunctionValue);
	}

	dVector pScaled((int)p.size());
	for (int i=0; i<p.size(); i++)
		pScaled[i] = (p[i] - pMin[i]) / (pMax[i] - pMin[i]);

	arFunVals = cmaes_init(&evo, (int)p.size(), &pScaled[0], &initialStdDev[0], 0, populationSize, "non");
	evo.sp.stopMaxFunEvals = 1e299;
	evo.sp.stStopFitness.flg = 1;
	evo.sp.stStopFitness.val = solveFunctionValue;
	evo.sp.stopMaxIter = DBL_MAX;
	evo.sp.stopTolFun = 1e-9;
	evo.sp.stopTolFunHist = solveHistToleranceValue;
	evo.sp.stopTolX = 1e-11;
	evo.sp.stopTolUpXFactor = 1e3;
	evo.sp.seed = 0;
	
	for (int i = 0;i < pScaled.size();i++)
		evo.rgxbestever[i] = pScaled[i];
	evo.rgxbestever[p.size()] = initialFunctionValue;
	evo.rgxbestever[p.size()+1] = 1;

	assert(IS_EQUAL(cmaes_Get(&evo, "fbestever"), initialFunctionValue));

	totalNumberOfIterations = 0;
	initialized = true;
}

bool CMAIterativeFunctionMinimizer::step(int numIterations)
{
	if (!initialized)
		initialize();

	int iter=0;
	for (; iter<numIterations && !cmaes_TestForTermination(&evo); ++iter)
	{
		double *const *pop = cmaes_SamplePopulation(&evo);
		int popSize = (int)cmaes_Get(&evo, "popsize");
		
		if (printLevel >= 2) {
			Logger::logPrint("CMA: Iteration %d (total %d)\n", iter, iter+totalNumberOfIterations);
		}
		
		for (int popIdx=0; popIdx<popSize; popIdx++)
		{
			for (int i=0; i<p.size(); i++)
				p[i] = (1 - pop[popIdx][i])*pMin[i] + (pop[popIdx][i])*pMax[i];
			
			// Evaluate the objective for each sampling point.
			arFunVals[popIdx] = function->computeValue(p);
		}

		cmaes_UpdateDistribution(&evo, arFunVals);

		// Print output
		if (printLevel >= 2)
		{
			dVector pTmp((int)p.size(),0);
			dVector pi((int)p.size(), 0);

			cmaes_GetInto(&evo, "xmean", &pTmp[0]);
			for (int i=0; i<p.size(); i++)
				pi[i] = (1 - pTmp[i])*pMin[i] + (pTmp[i])*pMax[i];
			double mean = function->computeValue(pi);

			cmaes_GetInto(&evo, "xbest", &pTmp[0]);
			for (int i=0; i<p.size(); i++)
				pi[i] = (1 - pTmp[i])*pMin[i] + (pTmp[i])*pMax[i];
			double best = function->computeValue(pi);

			cmaes_GetInto(&evo, "xbestever", &pTmp[0]);
			for (int i=0; i<p.size(); i++)
				pi[i] = (1 - pTmp[i])*pMin[i] + (pTmp[i])*pMax[i];
			double bestEver = function->computeValue(pi);

			Logger::logPrint("       Mean function value: %.6lf\n", mean);
			Logger::logPrint("       Best function value: %.6lf\n", best);
			Logger::logPrint("   Bestever function value: %.6lf\n", bestEver);
		}
	}

	totalNumberOfIterations += iter;
	return iter < numIterations;
}

double CMAIterativeFunctionMinimizer::getCurrentBestSolution(dVector &p)
{
	if (initialized)
	{
		p.resize(this->p.size());
		cmaes_GetInto(&evo, "xbestever", &p[0]);
		for (int i=0; i<p.size(); i++)
			p[i] = (1-p[i])*pMin[i] + (p[i])*pMax[i];
		return cmaes_Get(&evo, "fbestever");
	}
	else
	{
		p = this->p;
		if (function && p.size()!=0) {
			return function->computeValue(p);
		}
	}
		
	return DBL_MAX;
}

void CMAIterativeFunctionMinimizer::getCurrentMean(dVector &p)
{
	p.resize(this->p.size());
	cmaes_GetInto(&evo, "xmean", &p[0]);
	for (int i=0; i<p.size(); i++)
		p[i] = (1-p[i])*pMin[i] + (p[i])*pMax[i];
}

void CMAIterativeFunctionMinimizer::getCurrentStdDev(dVector &stdDev)
{
	stdDev.resize(this->p.size());
	cmaes_GetInto(&evo, "stddev", &stdDev[0]);
}

void CMAIterativeFunctionMinimizer::setCurrentMean(const dVector &mean)
{
	for (int i=0; i<p.size(); i++)
		evo.rgxmean[i] = mean[i];
}

void CMAIterativeFunctionMinimizer::multiplyStdDevBy(double factor)
{
	evo.sigma *= factor;
}

