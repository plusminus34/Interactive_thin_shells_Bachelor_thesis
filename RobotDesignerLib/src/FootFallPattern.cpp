#include <RobotDesignerLib/FootFallPattern.h>
#include <MathLib/mathLib.h>

FootFallPattern::FootFallPattern(void){
	strideSamplePoints = 12;
}

FootFallPattern::~FootFallPattern(void){
}

int FootFallPattern::getStepPatternIndexForLimb(GenericLimb* limb) {
	for (uint i=0;i<stepPatterns.size();i++)
		if (stepPatterns[i].limb == limb)
			return i;

	return -1;
}

StepPattern* FootFallPattern::getStepPatternForLimb(GenericLimb* limb) {
	for (uint i=0;i<stepPatterns.size();i++)
		if (stepPatterns[i].limb == limb)
			return &stepPatterns[i];

	return NULL;
}

void FootFallPattern::addStepPattern(GenericLimb* limb, int start, int end){
	if (end < start)
		end += strideSamplePoints;
	stepPatterns.push_back(StepPattern(limb, start, end));
}

void FootFallPattern::doubleNumberOfSamples(){
	int oldSampleCount = strideSamplePoints;
	int newSampleCount = strideSamplePoints*2;
	strideSamplePoints = newSampleCount;

	for(uint i = 0; i < stepPatterns.size(); i++)
	{
		stepPatterns[i].startIndex *= 2;
		stepPatterns[i].endIndex = stepPatterns[i].endIndex*2+1;
	}
}

