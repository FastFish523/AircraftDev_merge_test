//
// Created by MikuSoft on 2025/11/11.
// Copyright (c) 2025 JiuTianAoXiang All rights reserved.
//

#pragma once 

// region Include
// region STL
// endregion
// region ThirdParty
#include"Eigen/Dense"
// endregion
// region Self
#include "Dynamics.h"
#include "State.h"
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Utils/Kinematics"
#if defined(_WIN32) && !defined(StaticUtils_Build)
#ifdef SharedUtils_Build
#define Dll_Export_Import __declspec(dllexport)
#else
#define Dll_Export_Import __declspec(dllimport)
#endif
#else
#define Dll_Export_Import
#endif
// endregion

namespace ModelDevelop::Utils {
class Dll_Export_Import Kinematics {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
    Kinematics() = default;

    ~Kinematics() = default;

// endregion

// region Public Attributes
// endregion

// region Public Methods
    D_State cal_d_state(const State& state, double mass,const Eigen::Matrix3d& inertia,const Eigen::Vector3d& P_body,const Eigen::Vector3d& M_body,const Eigen::Vector3d& rudder,double s,double l,double b);




// endregion

// region Get/Set选择器
    public:
    Dynamics _dynamics{};
// endregion

// region Private Attributes
    private:

// endregion

// region Private Methods
    private:


// endregion
};
}
#undef Dll_Export_Import
#undef PRETTY_FILE_NAME
