//
// Created by MikuSoft on 2025/12/30.
// Copyright (c) 2025 JiuTianAoXiang All rights reserved.
//
// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "Atmosphere.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "Environment//Atmosphere"
// endregion

// region Using NameSpace
// endregion

namespace Environment::Utils {
// region Static Attributes Init
// endregion

// region USING/FRIEND
// endregion

// region Constructor
    Atmosphere::Atmosphere() {
        _flags  = std::make_shared<nrlmsise_flags>();
        _input  = std::make_shared<nrlmsise_input>();
        _output = std::make_shared<nrlmsise_output>();
    }

    double Atmosphere::getDensity(const int year, const int doy, const double hours, const double lon, const double lat, const double alt, const double f107,
                                  const double f107a, const double ap) {
        _input->year   = year;                      // 年
        _input->doy    = doy;                       // 年积日 (day of year)
        _input->sec    = hours*3600;                   // 秒 (12:00:00)
        _input->g_long = lon;                       // 地理经度
        _input->g_lat  = lat;                       // 地理纬度
        _input->alt    = alt;                       // 高度 (m)

        _input->lst    = calculateLST(hours, lon); // 地方时
        _input->f107   = f107;                      // F10.7指数
        _input->f107A  = f107a;                     // F10.7平均值
        _input->ap     = ap;                        // 磁情指数

        for (int &_switch: _flags->switches) {
            _switch = 1;
        }
        gtd7(_input.get(), _flags.get(), _output.get());
        const auto density = _output->d[5];
        return density;
    }

    double Atmosphere::getTemperature(const int year, const int doy, const double hours, const double lon, const double lat, const double alt, const double f107,
                                      const double f107a, const double ap) {
        _input->year   = year;                      // 年
        _input->doy    = doy;                       // 年积日 (day of year)
        _input->sec    = hours*3600;                   // 秒 (12:00:00)
        _input->g_long = lon;                       // 地理经度
        _input->g_lat  = lat;                       // 地理纬度
        _input->alt    = alt;                       // 高度 (m)
        _input->lst    =  calculateLST(hours, lon); // 地方时
        _input->f107   = f107;                      // F10.7指数
        _input->f107A  = f107a;                     // F10.7平均值
        _input->ap     = ap;                        // 磁情指数

        for (int &_switch: _flags->switches) {
            _switch = 1;
        }
        gtd7(_input.get(), _flags.get(), _output.get());
        const auto tem = _output->t[1];
        return tem;
    }

// endregion

// region Public Methods

// endregion

// region Get/Set选择器
// endregion

// region Private Methods
// endregion
}
#undef PRETTY_FILE_NAME
