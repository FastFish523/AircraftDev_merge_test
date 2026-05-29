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
#include "DateTime.h"
#include "TimeAccuracy.h"
#include "TimeSpan.h"
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Utils/CJulian"
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
class Dll_Export_Import CJulian {
/// region 构造函数
    public:
        CJulian() noexcept: CJulian(0, 0, 0, 0, 0, 0, 0, 0) {};

        [[maybe_unused]]
        CJulian(uint64_t time, TimeAccuracy timeAccuracy);

        [[maybe_unused]]
        explicit CJulian(double time, TimeAccuracy timeAccuracy);

        [[maybe_unused]]
        explicit CJulian(const DateTime &dateTime);

        [[maybe_unused]]
        CJulian(int32_t year, double day);

        CJulian(uint32_t year, uint32_t month, uint32_t day,
                uint32_t hour, uint32_t minute, uint32_t second,
                uint32_t milliSecond, uint32_t microSecond);

        ~CJulian() = default;

        auto operator=(const CJulian &other) -> CJulian &;
// endregion

// region 公有属性
    public:
// endregion

// region 公有方法
    public:
        /*!
         * Greenwich Mean Sidereal Time
         * @return
         */
        [[nodiscard]]
        auto toGMST() const -> double;

        /*!
         * Local Mean Sideral Time
         * @param lon
         * @return
         */
        [[nodiscard]]
        [[maybe_unused]]
        auto toLMST(double lon) const -> double;

        /*!
         * To time_t type - avoid using
         * @return
         */
        [[nodiscard]]
        [[maybe_unused]]
        auto toTime() const -> DateTime;

        [[nodiscard]]
        auto getDate() const -> double;

        [[nodiscard]]
        [[maybe_unused]]
        auto FromJan1_00h_1900() const -> double;

        [[nodiscard]]
        [[maybe_unused]]
        auto FromJan1_12h_1900() const -> double;

        [[nodiscard]]
        auto FromJan1_12h_2000() const -> double;

        [[nodiscard]]
        auto spanDay(const CJulian &b) const -> double;

        [[nodiscard]]
        [[maybe_unused]]
        auto spanHour(const CJulian &b) const -> double;

        [[nodiscard]]
        [[maybe_unused]]
        auto spanMin(const CJulian &b) const -> double;

        [[nodiscard]]
        [[maybe_unused]]
        auto spanSec(const CJulian &b) const -> double;

        auto getComponent(int32_t &pYear, int32_t &pMon, double &pDOM) const -> void;

        [[maybe_unused]]
        auto addDay(double day) -> void;

        [[maybe_unused]]
        auto addHour(double hr) -> void;

        [[maybe_unused]]
        auto addMin(double min) noexcept -> void;

        [[maybe_unused]]
        auto addSec(double sec) noexcept -> void;
// endregion

// region Get/Set选择器
    public:
// endregion

// region 私有变量
    private:
        double m_Date = 0; // Julian date
        const double EPOCH_JAN1_00H_1900 = 2415019.5; // Jan 1.0 1900 = Jan 1 1900 00h UTC
        const double EPOCH_JAN1_12H_1900 = 2415020.0; // Jan 1.5 1900 = Jan 1 1900 12h UTC
        const double EPOCH_JAN1_12H_2000 = 2451545.0; // Jan 1.5 2000 = Jan 1 2000 12h UTC
        const double HR_PER_DAY = 24.0;          // Hours per day   (solar)
        const double MIN_PER_DAY = 1440.0;        // Minutes per day (solar)
        const double SEC_PER_DAY = 86400.0;       // Seconds per day (solar)
// endregion

// region 私有方法
    private:
        void Initialize(uint32_t year, double day);
// endregion
};
}
#undef Dll_Export_Import
#undef PRETTY_FILE_NAME