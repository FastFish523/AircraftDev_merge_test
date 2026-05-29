//
// Created by MikuSoft on 2025/11/11.
// Copyright (c) 2025 JiuTianAoXiang All rights reserved.
//

#pragma once 

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Utils/Constants"
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
class Dll_Export_Import Constants {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
    Constants() = default;

    ~Constants() = default;

// endregion

// region Public Attributes
    // 地球参数
    static constexpr double EARTH_RADIUS = 6378137.0;        // 地球半径(m)
    static constexpr double EARTH_GM = 3.986004418e14;       // 地球引力常数(m^3/s^2)
    static constexpr double EARTH_OMEGA = 7.292115e-5;       // 地球自转角速度(rad/s)

    // 数学常数
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double DEG_TO_RAD = PI / 180.0;
    static constexpr double RAD_TO_DEG = 180.0 / PI;


    static inline double DEF_AL_2      = 6378137.0;              // 地球半径，单位：m
    static inline double DEF_AL_3      = 6356752.3142;           // 半短轴，单位：m
    static inline double DEF_E2_WGS84  = 6.69437999014E-3;       // WGS84地球椭球体第一偏心率的平方
    static inline double DEF_EP2_WGS84 = 6.73949674228E-3;       // WGS84地球椭球体第二偏心率的平方

    // 大气参数
    static constexpr double R_GAS = 287.05;                 // 空气气体常数(J/kg/K)
    static constexpr double GAMMA_AIR = 1.4;                // 空气比热比

    // WGS84椭球参数
    static constexpr double EARTH_FLATTENING = 1.0 / 298.257223563;  // 扁率
    static constexpr double EARTH_SEMI_MINOR_AXIS = EARTH_RADIUS * (1.0 - EARTH_FLATTENING); // 短半轴
    static constexpr double EARTH_ECCENTRICITY_SQ = 1.0 - (EARTH_SEMI_MINOR_AXIS * EARTH_SEMI_MINOR_AXIS)
                                                   / (EARTH_RADIUS * EARTH_RADIUS); // 第一偏心率平方
// endregion

// region Public Methods
    public:
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