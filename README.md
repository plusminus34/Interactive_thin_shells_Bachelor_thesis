# Simulation and Control Playground

# Table of Contents
- [Getting Started](#getting-started)
- [Compilation](#compilation)
  - [Thirdparty Libraries](#thirdparty-libraries)

## Getting Started
### Windows
If you are on Windows, you can use the `easyFirstSetup.cmd` script. It gets the required thirdparty dependencies and runs cmake.

0. (clone this repository)
1. Install Visual Studio: https://www.visualstudio.com/
2. Install CMake: https://cmake.org/download/
3. Run `easyFirstSetup.cmd` (e.g. by double clicking the script)

Now there is a folder `build` where you can find the Visual Studio solution `SCP.sln`. Open it in Visual Studio and build!

### Mac OS and Linux
There is no `easyFirstSetup` script for Mac OS or Linux. You will have to obtain and compile the thirdparty libraries yourself, following the guide below.

## Build
This section describes how to get the thirdparty libraries and set up the build system. The build system of SCP uses CMake and git submodules (where possible). Libraries that are not cloned via git submodules need to be obtained and compiled by the user, as described below.

First make sure you have git and cmake:

- CMake: Install latest release (> 3.5) from https://cmake.org/download/
- Git: Install from https://git-scm.com/downloads or use a 

### Thirdparty Libraries

#### Windows
> Note: For Windows users, there is a convenience repository that contains all the required precompiled thirdparty libraries: https://bitbucket.org/scoros/libs. You can clone this repository and put it in the same folder as your SCP repository: `folder/scp`, `folder/libs`. The CMake scripts will always first check the `folder/libs` for thirdparty libraries.

#### Linux (Mac OS, untested)
This guide will be for Ubuntu or other debian-based distros.

#### Graphics
Depending on your system, you should install the correct OpenGL libraries, e.g.:
`sudo apt-get install xorg-dev libglu1-mesa-dev`

##### Blas / Lapack
`sudo apt install libblas3 libblas-dev liblapack3 liblapack-dev`

##### Gfortran
`sudo apt install gfortran`

##### GTest
`sudo apt install libgtest-dev`

Build the gtest libraries and copy them to `/usr/local/lib/`:

```bash
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
# copy or symlink libgtest.a and libgtest_main.a to your /usr/local/lib folder
sudo cp *.a /usr/local/lib
```

##### MA27
1. Get MA27 from http://www.hsl.rl.ac.uk/download/MA27/1.0.0/a/
2. Build it:

```bash
./configure
make 
sudo make install
```

##### OOQP
1. Get OOQP from http://pages.cs.wisc.edu/~swright/ooqp/download/
2. Apply OOQP hash patch:
   1. Get it from https://gitlab.ethz.ch/snippets/24 (click on the download icon, save as `OOQP_hash.patch`)
   2. Apply patch to OOQP folder: `patch -p0 < OOQP_hash.patch`
3. Build OOQP:

```bash
MA27LIB=/usr/local/lib/libma27.a ./configure
make
sudo make install
```

##### ODE
1. Get ode-0.13 from https://sourceforge.net/projects/opende/files/ODE/0.13/.
2. 	Build it:

```bash
 	CXXFLAGS=-fpermissive ./configure
   	make
   	sudo make install
```

##### FreeType  (https://www.freetype.org/):
`sudo apt install libfreetype6 libfreetype6-dev`

### Compilation
1. Clone this repository
2. Run `git submodule update --init --recursive`
3. Generate build files. In the cloned folder do:

```bash
mkdir build && cd build
cmake -DNANOGUI_USE_GLAD=1 ..
```

`NANOGUI_USE_GLAD` makes sure we use glad.

4. Compile

```bash
make -j12
```

> Notes: If your GPU driver can't create a OpenGL context in compatability mode, run CMake with `SCP_GUI_TWO_WINDOWS=1 cmake ..`