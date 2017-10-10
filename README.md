# Simulation and Control Playground

## The fast and easy way
If you are on Windows, you can skip everything below and do the following:

0. (It is assumed you have git installed and cloned this repository.)
1. Install CMake: https://cmake.org/download/
2. Run `easyFirstSetup.cmd` (e.g. by double clicking the script)

## Compilation

The build system of SCP uses CMake and git submodules (where possible). 

### Prerequisites

__CMake__ (https://cmake.org/):

Install latest release from https://cmake.org/download/

__SCP Libs__
- Windows:
  clone `https://bitbucket.org/scoros/libs` to the same directory as the SCP repository is in. We will call this `path\to\libs`.

__ODE__
- Windows:
  ODE is part of the libs reposiroy (`https://bitbucket.org/scoros/libs`). Add the location of `ode.dll` to the PATH environment variable:
  1. Hit the Windows key, type "environment variables", choose "Edit Environment Variables"
  2. Choose the Path variable, and append the the following path:
    `path\to\libs\thirdPartyCode\ode-0.13\lib\ReleaseDLL`
  3. Click `Ok` twice

__FreeType__ (https://www.freetype.org/):
- Windows:
  Install from http://gnuwin32.sourceforge.net/packages/freetype.htm
  Get the setup and run it (may have to turn off Windows Defender or other Antivirus)
  Set the environment variable `FREETYPE_DIR` to the install path, e.g. `C:\Program Files (x86)\GnuWin32`
- Linux/OSX/Windows:
  Download source from https://www.freetype.org/ and compile.
  Set the environment variable `FREETYPE_DIR` to the install path

More info about the cmake find script can be found [here](https://cmake.org/cmake/help/v3.0/module/FindFreetype.html).

### Compilation
1. after cloning to `path/to/scp`, run `git submodule update --init --recursive`
2. Make a folder `build` in `path/to/scp` (`path/to/scp/build`).
3. Run `cmake ..` in `path/to/scp/build` to generate build files.
4. Compile using the build files generated in `path/to/scp/build`.