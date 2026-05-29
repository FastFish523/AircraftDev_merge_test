//
// Created by MikuSoft on 2025/11/11.
// Copyright (c) 2025 JiuTianAoXiang All rights reserved.
//
// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "CJulian.h"
#include <cmath>
#include <cassert>
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Utils/CJulian"
// endregion

// region Using NameSpace
// endregion

namespace ModelDevelop::Utils {

// region 构造函数
    [[maybe_unused]]
    CJulian::CJulian(uint64_t time, const TimeAccuracy timeAccuracy) {
        if (timeAccuracy == TimeAccuracy::Second) {
            time = time * 1000 * 1000;
        } else if (timeAccuracy == TimeAccuracy::Millisecond) {
            time = time * 1000;
        }
        const auto dateTime = DateTime(time, TimeAccuracy::Microsecond);
        *this = CJulian(dateTime);
    }

    [[maybe_unused]]
    CJulian::CJulian(const double time, const TimeAccuracy timeAccuracy) {
        const auto dateTime = DateTime(time, timeAccuracy);
        *this = CJulian(dateTime);
    }

    [[maybe_unused]]
    CJulian::CJulian(const DateTime &dateTime) {
        *this = CJulian(dateTime.year(),
                        dateTime.month(),
                        dateTime.day(),
                        dateTime.hour(),
                        dateTime.minute(),
                        dateTime.second(),
                        dateTime.millisecond(),
                        dateTime.microsecond());
    }

    [[maybe_unused]]
    CJulian::CJulian(const int year, const double day) {
        m_Date = 0;
        Initialize(year, day);
    }

    CJulian::CJulian(const uint32_t year, const uint32_t month, const uint32_t day,
                     const uint32_t hour, const uint32_t minute, const uint32_t second,
                     const uint32_t milliSecond, const uint32_t microSecond) {
        m_Date = 0;
        // Calculate N, the day of the year (1..366)
        int N;
        const auto F1 = static_cast<int32_t>((275.0 * month) / 9.0);
        const auto F2 = static_cast<int32_t>((month + 9.0) / 12.0);

        if (DateTime::isLeapYear(year)) {
            // Leap year
            N = F1 - F2 + static_cast<int32_t>(day) - 30;
        } else {
            // Common year
            N = F1 - (2 * F2) + static_cast<int32_t>(day) - 30;
        }

        const double dblDay = N + (static_cast<double>(hour) +
                                   (static_cast<double>(minute) +
                                    (static_cast<double>(second) +
                                     (static_cast<double>(milliSecond) +
                                      static_cast<double>(microSecond) / 1000.0) / 1000.0) / 60.0) / 60.0) / 24.0;

        Initialize(year, dblDay);
    }

    auto CJulian::operator=(const CJulian &other) -> CJulian & {
        if (this != &other) {
            this->m_Date = other.m_Date;
        }
        return *this;
    }
// endregion

// region 公有方法

    [[nodiscard]]
    auto CJulian::toGMST() const -> double {
        constexpr double OMEGA_E = 1.00273790934;
        const double UT = fmod(m_Date + 0.5, 1.0);
        const double TU = (FromJan1_12h_2000() - UT) / 36525.0;

        double GMST = 24110.54841 + TU *
                                    (8640184.812866 + TU * (0.093104 - TU * 6.2e-06));

        GMST = fmod(GMST + SEC_PER_DAY * OMEGA_E * UT, SEC_PER_DAY);

        if (GMST < 0.0)
            GMST += SEC_PER_DAY;  // "wrap" negative modulo value

        return (2 * 3.141592653 * (GMST / SEC_PER_DAY));
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto CJulian::toLMST(const double lon) const -> double {
        return fmod(toGMST() + lon, 2 * 3.141592653);
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto CJulian::toTime() const -> DateTime {
        int nYear;
        int nMonth;
        double dblDay;

        getComponent(nYear, nMonth, dblDay);

        // dblDay is the fractional Julian Day (i.e., 29.5577).
        // Save the whole number day in nDOM and convert dblDay to
        // the fractional portion of day.
        const int nDOM = static_cast<int>(dblDay);

        dblDay -= nDOM;

        constexpr int SEC_PER_MIN = 60;
        constexpr int SEC_PER_HR  = 60 * SEC_PER_MIN;


        const int secs = static_cast<int32_t>(floor((dblDay * SEC_PER_DAY) + 0.5));

        return {nYear, static_cast<uint32_t>(nMonth), static_cast<uint32_t>(nDOM),
                static_cast<uint32_t>(secs) / SEC_PER_HR, static_cast<uint32_t>(secs % SEC_PER_HR) / SEC_PER_MIN,
                static_cast<uint32_t>(secs % SEC_PER_HR) % SEC_PER_MIN};
    }

    auto CJulian::getDate() const -> double {
        return m_Date;
    }

    [[maybe_unused]]
    auto CJulian::FromJan1_00h_1900() const -> double {
        return m_Date - EPOCH_JAN1_00H_1900;
    }

    [[maybe_unused]]
    auto CJulian::FromJan1_12h_1900() const -> double {
        return m_Date - EPOCH_JAN1_12H_1900;
    }

    auto CJulian::FromJan1_12h_2000() const -> double {
        return m_Date - EPOCH_JAN1_12H_2000;
    }

    auto CJulian::spanDay(const CJulian &b) const -> double {
        return m_Date - b.m_Date;
    }

    [[maybe_unused]]
    auto CJulian::spanHour(const CJulian &b) const -> double {
        return spanDay(b) * HR_PER_DAY;
    }

    [[maybe_unused]]
    auto CJulian::spanMin(const CJulian &b) const -> double {
        return spanDay(b) * MIN_PER_DAY;
    }

    [[maybe_unused]]
    auto CJulian::spanSec(const CJulian &b) const -> double {
        return spanDay(b) * SEC_PER_DAY;
    }

    void CJulian::getComponent(int32_t &pYear, int32_t &pMon, double &pDOM) const {
        const double jdAdj = getDate() + 0.5;
        const int Z        = static_cast<int>(jdAdj); // integer part
        const double F     = jdAdj - Z;               // fractional part
        const double alpha = static_cast<int>((Z - 1867216.25) / 36524.25);
        const double A     = Z + 1 + alpha - static_cast<int>(alpha / 4.0);
        const double B     = A + 1524.0;
        const int C        = static_cast<int>((B - 122.1) / 365.25);
        const int D        = static_cast<int>(C * 365.25);
        const int E        = static_cast<int>((B - D) / 30.6001);

        const double DOM = B - D - static_cast<int>(E * 30.6001) + F;
        const int month  = (E < 13.5) ? (E - 1) : (E - 13);
        const int year   = (month > 2.5) ? (C - 4716) : (C - 4715);

        pYear = year;
        pMon = month;
        pDOM = DOM;
    }

    [[maybe_unused]]
    void CJulian::addDay(const double day) {
        m_Date += day;
    }

    [[maybe_unused]]
    void CJulian::addHour(const double hr) {
        m_Date += (hr / HR_PER_DAY);
    }

    [[maybe_unused]]
    void CJulian::addMin(const double min) noexcept {
        m_Date += (min / MIN_PER_DAY);
    }

    [[maybe_unused]]
    void CJulian::addSec(const double sec) noexcept {
        m_Date += (sec / SEC_PER_DAY);
    }
// endregion

// region Get/Set选择器
// endregion

// region 私有方法
    void CJulian::Initialize(uint32_t year, const double day) {
        // 1582 A.D.: 10 days removed from calendar
        // 3000 A.D.: Arbitrary error checking limit
        assert((year > 1582) && (year < 3000));
        assert((day >= 1.0) && (day < 367.0));

        // Now calculate Julian date

        year--;

        // Centuries are not leap years unless they divide by 400
        const int A = static_cast<int>(year / 100);
        const int B = 2 - A + (A / 4);

        const double NewYears = static_cast<int32_t>(365.25 * year) +
                                static_cast<int32_t>(30.6001 * 14) +
                                1720994.5 + B;  // 1720994.5 = Oct 30, year -1

        m_Date = NewYears + day;
    }
}
#undef PRETTY_FILE_NAME