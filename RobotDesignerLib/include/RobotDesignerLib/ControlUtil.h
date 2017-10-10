#pragma once
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>


Quaternion getHeadingOffsetFromMotionPlanToRobotState(LocomotionEngineMotionPlan* mp, double mpPhase, Robot* robot);
