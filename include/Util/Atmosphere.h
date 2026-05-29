//
// Created by MikuSoft on 2025/12/30.
// Copyright (c) 2025 JiuTianAoXiang All rights reserved.
//

#pragma once

// region Include
// region STL
// endregion
// region ThirdParty
#include <memory>

#include "Util/nrlmsise-00.h"
// endregion
// region Self
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "Environment//Atmosphere"
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

namespace Environment::Utils {
    class Dll_Export_Import Atmosphere {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
        Atmosphere();

        ~Atmosphere() = default;

// endregion

// region Public Attributes
    public:

// endregion

// region Public Methods
    public:
        double getDensity(int year, int doy, double hours, double lon, double lat, double alt, double f107 = 150, double f107a = 150, double ap = 4);
        double getTemperature(int year, int doy, double hours, double lon, double lat, double alt, double f107 = 150, double f107a = 150, double ap = 4);

// endregion

// region Get/Set选择器
    public:
// endregion

// region Private Attributes
    private:
        std::shared_ptr<nrlmsise_input> _input = nullptr;
        std::shared_ptr<nrlmsise_flags> _flags = nullptr;
        std::shared_ptr<nrlmsise_output> _output = nullptr;
// endregion

// region Private Methods
    private:
        // 计算地方视太阳时 (Local Solar Time)
        double calculateLST(const double utc_hours, const double longitude_deg) {
            // 简化的地方时计算
            // LST = UTC + 经度/15
            double lst = utc_hours + longitude_deg / 15.0;

            // 归一化到[0, 24)
            while (lst >= 24.0) lst -= 24.0;
            while (lst < 0.0) lst += 24.0;

            return lst;
        }


// endregion
    };
}
#undef Dll_Export_Import
#undef PRETTY_FILE_NAME
