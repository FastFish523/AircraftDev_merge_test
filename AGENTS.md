# AGENTS.md

## Architecture Overview
This is a C++ missile simulation framework using modular components:
- **Missile** (TGC/TGCMissile.h): Main simulation class integrating subsystems
- **Guidance** (TGC/Guidance.h): Computes line-of-sight and acceleration commands
- **Control** (TGC/Control.h): PID control for rudder deflections and moments
- **Engine** (TGC/Engine.h): Thrust and mass depletion modeling
- **IMU** (TGC/IMU.h): Inertial measurement simulation
- **Kinematics** (Util/Kinematics.h): 6DOF dynamics integration using RK4
- **Aerodynamics** (Util/Aerodynamics.h): Force/moment coefficients via NRLMSISE atmosphere

Data flows from target inputs → guidance → control → RK4 state update → IMU → file logging.

## Key Workflows
- **Build**: Use CMake in `cmake-build-debug-visual-studio/`. Run `cmake --build . --config Debug` for MSVC or `make` on Linux.
- **Test**: Execute `TGC_Test.exe` (links Missile lib) for simulation runs; outputs to `Results/TGC/`.

  - On Windows (recommended) a reproducible configure/build sequence is:
	```powershell
	cmake -S . -B cmake-build-debug-visual-studio -G "Ninja" -DCMAKE_BUILD_TYPE=Debug
	cmake --build cmake-build-debug-visual-studio --config Debug
	```
	The actual binaries and libraries are written to `_Build/out/<platform>/<configuration>/` (CMake sets `OutputPath` to `_Build/out`). For MSVC the resolved path is typically `_Build/out/windows/Debug/`.
- **Debug**: Attach VS debugger to TGC_Test.exe; step through `Missile::update()` RK4 loop.

## Project Conventions
- **Coordinates**: ECF (Earth-Centered Fixed) primary; convert via `CoordinateHelper` to LLA/NUE/body.
- **Units**: Radians internally; getters return degrees (*57.3). Eigen vectors/matrices for math.
- **Aero Models**: Custom lambda in `Missile` constructor (e.g., CN = 0.3 + 0.6*Ma²/(1+0.8*Ma⁴) + 4/sqrt(1+(Ma²-1)²)).
- **File Structure**: Headers in `include/`, sources in `src/`, tests in `Test/`. Use `#pragma once` and region comments.
- **Dependencies**: Eigen3 for linear algebra; NRLMSISE-00 for atmosphere; ylt for utilities.
  - CMake feature flags used by this project (see top-level `CMakeLists.txt`): `-DIS_LXN_TEST=1` to enable test macros, `-DIS_LOG_PRINT=1` to enable extra log printing.
  - The project supports building TGC as shared or static. See `include/TGC/TGCMissile.h` for the `DLL_EXPORT_IMPORT` macro and the `SharedTGC_Build` / `StaticTGC_Build` symbols used on Windows.

## Integration Points
- **MatlabHelper**: Aero coefficients for specific missiles (e.g., `AGM86C/aero.m`) computed externally and hardcoded in lambdas.
 - **External Libs**: Link Eigen via `include/ThirdParty/eigen3/`; build shared libs (TGC.dll, Utils.dll) to `_Build/out/`.
 - **Test runner**: `Test/TGC/CMakeLists.txt` creates the `TGC_Test` executable (see `Test/TGC/main.cpp` for a standard full-mission console runner).
 - **Runtime outputs**: Simulation output files are written by `TGC::FileSaver` into `./Results/TGC/` (instantiated in `src/TGC/TGCMissile.cpp`).

Reference: `TGCMissile.cpp` update() for simulation loop; `CommonStructs.h` for data types.</content>
<parameter name="filePath">F:\BUAA\1122\AircraftModelBase\ModelDev\ModelDev\AGENTS.md
