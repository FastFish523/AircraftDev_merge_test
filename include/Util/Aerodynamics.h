//
// Created by MikuSoft on 2025/11/11.
// Copyright (c) 2025 JiuTianAoXiang All rights reserved.
//

#pragma once

// region Include
// region STL
// endregion
// region ThirdParty
#include <Atmosphere.h>

#include "State.h"
#include "Eigen/Dense"
// endregion
// region Self
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Utils/Aerodynamics"
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
    class Dll_Export_Import Aerodynamics {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
        Aerodynamics()=default;

        ~Aerodynamics() = default;

// endregion

// region Public Attributes
// endregion

// region Public Methods
        Eigen::Vector3d getF(const State& state,const Eigen::Vector3d& rudder,double s,double l,double b);
        Eigen::Vector3d getM(const State& state,const Eigen::Vector3d& rudder,double s,double l,double b);

        // 计算大气密度（简化指数模型）
        static double calculateAtmosphereDensity(double altitude);


        //计算力和力矩系数

        std::function<void(double, double, double, double, double, double, double &, double &, double &, double &, double &, double &)> computeAeroCoefficientsCB;

// endregion

// region Get/Set选择器
    public:
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
