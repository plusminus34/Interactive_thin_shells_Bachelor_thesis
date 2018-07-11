cmake_minimum_required(VERSION 3.5)

project(Apps)

# fix for Visual Studio: copy dll's to bin directory 
# (otherwise VS doesn't find the dll ...)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Workaround for VS2015 bug
if (MSVC)
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /FORCE:Multiple")
endif()

add_subdirectory(OptimizationDemoApp)
add_subdirectory(FireworksApp)
add_subdirectory(PDEsDemoApp)
add_subdirectory(ShaderTestApp)
add_subdirectory(IKApp)
add_subdirectory(FEMSimApp)
add_subdirectory(TopOptApp)
add_subdirectory(FEMSim3DApp)
add_subdirectory(RBSimApp)
add_subdirectory(WheeledRobotSimApp)
add_subdirectory(FastRobotControlApp)
add_subdirectory(RobotDesignerApp)
add_subdirectory(PhysicalRobotControlApp)
add_subdirectory(BenderApp)
add_subdirectory(Plush)
add_subdirectory(PlotExampleApp)
add_subdirectory(CompliantRobotDesignApp)
add_subdirectory(KineSimApp)


