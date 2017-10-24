cmake_minimum_required(VERSION 3.5)

project(RobotDesignerLib)

# Workaround for VS2015 bug
if (MSVC)
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /FORCE:Multiple")
endif()

# add source files
file(GLOB sources
	${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME} STATIC ${sources})

include_directories("${EIGEN3_INCLUDE_DIR}")
include_directories("${CMAKE_SOURCE_DIR}/thirdparty/ft2.5.5")
include_directories("${CMAKE_SOURCE_DIR}/thirdparty")

# ODE
find_package(ODE REQUIRED)
include_directories(${ODE_INCLUDE_DIRS})

target_link_libraries(
	${PROJECT_NAME}
	PUBLIC
		MathLib
		OptimizationLib
		GUILib
		ControlLib
		RBSimLib
		BulletCollision
	PRIVATE
)

target_include_directories(
	${PROJECT_NAME} 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include
)

# group source files
source_group("Header Files\\Control"
	FILES
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ControlUtil.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/KinematicRobotController.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MOPTQPTrackingController.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/PositionBasedRobotController.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/RobotController.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/SimWindow.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/TorqueBasedRobotController.h
)
source_group("Source Files\\Control" 
	FILES
		${CMAKE_CURRENT_SOURCE_DIR}/src/ControlUtil.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/KinematicRobotController.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MOPTQPTrackingController.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/PositionBasedRobotController.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/RobotController.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/SimWindow.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/TorqueBasedRobotController.cpp
)

source_group("Header Files\\MOPT"
	FILES
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_COMTravelObjective.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_DynamicStabilityObjective.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_FeetPathSmoothnessObjective.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_FeetSlidingObjective.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_PeriodicRobotStateTrajectoriesObjective.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_RobotCOMObjective.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_RobotEndEffectorsObjective.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_RobotStateRegularizer.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_RobotTurningObjective.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_SmoothCOMTrajectories.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_SmoothRobotMotionTrajectories.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_SmoothStanceLegMotionObjective.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_StanceLegMotionRegularizer.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/FootFallPattern.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/LocomotionEngine.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/LocomotionEngineConstraints.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/LocomotionEngineEnergyFunction.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/LocomotionEngineMotionPlan.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_TorqueAngularAccelObjective.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_ForceAccelObjective.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/LocomotionEngineManager.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/LocomotionEngineManagerGRF.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/LocomotionEngineManagerIP.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_RobotCOMOrientationsObjective.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_COMTurningObjective.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_GRFSoftConstraints.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_NonLimbMotionRegularizer.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_NonLimbSmoothMotionObjective.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_RobotStateTransitionRegularizer.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_COMOrientationFluctuationRegularizer.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_EEPoseOffsetConstraintToInitial.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_PseudoLimbLengthConstraint.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_PseudoPeriodicEECOMPoseConstraint.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MPO_BarycentricWeightsRegularizerObjective.h
)

source_group("Source Files\\MOPT"
	FILES
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_COMTravelObjective.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_DynamicStabilityObjective.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_FeetPathSmoothnessObjective.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_FeetSlidingObjective.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_PeriodicRobotStateTrajectoriesObjective.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_RobotCOMObjective.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_RobotEndEffectorsObjective.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_RobotStateRegularizer.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_RobotTurningObjective.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_SmoothCOMTrajectories.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_SmoothRobotMotionTrajectories.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_SmoothStanceLegMotionObjective.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_StanceLegMotionRegularizer.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/FootFallPattern.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/LocomotionEngine.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/LocomotionEngineConstraints.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/LocomotionEngineEnergyFunction.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/LocomotionEngineMotionPlan.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_TorqueAngularAccelObjective.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_ForceAccelObjective.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/LocomotionEngineManager.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/LocomotionEngineManagerGRF.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/LocomotionEngineManagerIP.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_RobotCOMOrientationsObjective.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_COMTurningObjective.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_GRFSoftConstraints.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_NonLimbMotionRegularizer.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_NonLimbSmoothMotionObjective.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_RobotStateTransitionRegularizer.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_COMOrientationFluctuationRegularizer.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_EEPoseOffsetConstraintToInitial.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_PseudoLimbLengthConstraint.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_PseudoPeriodicEECOMPoseConstraint.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MPO_BarycentricWeightsRegularizerObjective.cpp
)

source_group("Header Files\\Design"
	FILES
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/FootFallPatternViewer.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ModuleDisplayWindow.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/RMC.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/RMCRobot.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/RMCJoint.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/RMCPin.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ModularDesignWindow.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/RMCBulletObject.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ParameterizedRobotDesign.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MOPTWindow.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MorphologicalRobotDesign.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/AbstractBulletObject.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/AbstractDesignWindow.h
)

source_group("Source Files\\Design"
	FILES
		${CMAKE_CURRENT_SOURCE_DIR}/src/FootFallPatternViewer.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/ModuleDisplayWindow.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/RMC.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/RMCRobot.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/RMCJoint.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/RMCPin.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/ModularDesignWindow.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/RMCBulletObject.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/ParameterizedRobotDesign.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MOPTWindow.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/MorphologicalRobotDesign.cpp
)

source_group("Header Files\\Living Connectors"
	FILES
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/LivingHornBracket.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/LivingBracketMotor.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/LivingMotor.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/LivingConnector.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/LivingSphereEE.h
	)

source_group("Source Files\\Living Connectors"
	FILES
		${CMAKE_CURRENT_SOURCE_DIR}/src/LivingHornBracket.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/LivingBracketMotor.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/LivingMotor.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/LivingConnector.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/LivingSphereEE.cpp
)