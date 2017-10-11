# vs to cmake todo

badly needed:
- [x] pixel ratio 
- [x] undef min/max
- [ ] ignore warning
- [x] put Cmake and dependcy folder on top
- [x] put it on gitlab.INF.ethz.ch

nice to have:
- [ ] ODE: 
  there’s a git! [https://bitbucket.org/odedevs/ode](https://bitbucket.org/odedevs/ode)include with git submodules?
  or put in “thirdparty” repo
  right now find script has a hint for ../libs/..... folder
- [ ] OOQP:
  put in thirdparty repo/google drive
  right now find script has a hint for ../libs/..... folder
- [ ] a lot of dlls have to be copied to build/bin/Debug build/bin/Release
  copy them with cmake!
- [ ] IKApp only works in release, otherwise some eigen error ...
    This might help: https://stackoverflow.com/questions/26459863/assertion-failed-eigen-debug-mode
- [ ] some libraries need to linked with debug/release ambiguition
    see here: https://stackoverflow.com/a/28742433

done:
- [x] in ControlLib and RobotDesignerLib: make VS filters like in original VS solution
- [x] for Debug, need to copy `freetype6.dll`  and  `zlib1.dll` from `C:\Program Files (x86)\GnuWin32\bin`
  --> users need to add to PATH
- [x] ode.dll: users should add ODE lib path to PATH
- [x] Enable parallel build (VS: "CXXFLAGS"=/MP)
- [x] need to update glad.h/.c manually
  --> create a git repo for our fork of nanogui!
- [x] data folder needs to be at build/Apps/data
  either:
  1. copy this with cmake! 
  2. use add_definitions(-DSCP_ROOT_DIR=${CMAKE_CURRENT_SOURCE_DIR}) as in here: https://stackoverflow.com/questions/9017573/define-preprocessor-macro-through-cmake
  --> Maybe better to change working directory: https://msdn.microsoft.com/en-us/library/bf7fwze1(v=vs.80).aspx
  <-- did this