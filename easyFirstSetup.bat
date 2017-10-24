@echo off

: Check if git command is available
where /q git
IF ERRORLEVEL 1 (
    ECHO Git is missing. Ensure it is installed and placed in your PATH.
    pause
    EXIT /B
) ELSE (
    ECHO Git found.
)

where /q cmake
IF ERRORLEVEL 1 (
    ECHO CMake is missing. Ensure it is installed and placed in your PATH.
    pause
    EXIT /B
) ELSE (
    ECHO CMake found.
) 

: Remember current dir
SET CURRENTDIR="%cd%"

: Let's clone all dependencies:
cd ..
IF NOT EXISTS libs (
    git clone https://gitlab.inf.ethz.ch/moritzge/libs/
    IF ERRORLEVEL 1 (
        ECHO Could not clone libs.
        pause
        EXIT /B
    ) ELSE (
        ECHO Libs cloned.
    ) 
) ELSE (
    cd libs
    git pull
    IF ERRORLEVEL 1 (
        ECHO Could not pull libs.
        pause
        EXIT /B
    ) ELSE (
        ECHO Libs pulled.
    ) 
)

: Update all submodules
cd %CURRENTDIR%
git submodule update --init --recursive
IF ERRORLEVEL 1 (
    ECHO Could not update submodules.
    pause
    EXIT /B
) ELSE (
    ECHO Submodules updated.
) 

: Create build folder and run cmake
cd %CURRENTDIR%
IF EXISTS build (
    rd /s /q build
)
mkdir build
cd build
IF ERRORLEVEL 1 (
    ECHO Could not create build folder.
    pause
    EXIT /B
) ELSE (
    ECHO Created build folder %cd%.
) 

cmake ..
IF ERRORLEVEL 1 (
    ECHO Did not run CMake successfully!
    pause
    EXIT /B
) ELSE (
    ECHO Build files generated.
) 

ECHO Success! You are now ready to use SCP :)

pause