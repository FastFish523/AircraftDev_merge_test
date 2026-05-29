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
#include "DateTime.h"

#include <array>
#include <chrono>
#include <climits>
#include <cmath>
#include <string>
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Utils/DateTime"
// endregion

// region Using NameSpace
// endregion

namespace ModelDevelop::Utils {
// region Static Attributes Init
// endregion

// region USING/FRIEND
// endregion
// region 构造函数
    DateTime::DateTime() {
        const uint64_t time = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        initializeTimeFromUTC(static_cast<int64_t>(time), TimeAccuracy::Microsecond);
    }

    DateTime::DateTime(double time, const TimeAccuracy timeAccuracy) {
        if (timeAccuracy == TimeAccuracy::Second) {
            time = time * 1000 * 1000;
        } else if (timeAccuracy == TimeAccuracy::Millisecond) {
            time = time * 1000;
        }
        initializeTimeFromUTC(static_cast<int64_t>(time), TimeAccuracy::Microsecond);
    }

    DateTime::DateTime(const uint64_t time, const TimeAccuracy timeAccuracy) {
        initializeTimeFromUTC(time, timeAccuracy);
    }

    DateTime::DateTime(const int32_t year, const uint32_t month, const uint32_t day,
                       const uint32_t hour, const uint32_t minute, const uint32_t second,
                       const uint32_t millisecond, const uint32_t microsecond) {
        const uint64_t time = getUTCTimeFromLocalTime(year, month, day, hour, minute, second, millisecond, microsecond);
        initializeTimeFromUTC(time, TimeAccuracy::Microsecond);
    }
// endregion

// region 公有方法
// region operator
    auto operator<<(std::ostream &os, const DateTime &dateTime) -> std::ostream & {
        std::array<char, 128> retVal{};
        sprintf(retVal.data(),
                "%d-%02d-%02d %02d:%02d:%02d.%03d%03d",
                dateTime._innerYear,
                dateTime._innerMonth,
                dateTime._innerTimeSpan.day(),
                dateTime._innerTimeSpan.hour(),
                dateTime._innerTimeSpan.minute(),
                dateTime._innerTimeSpan.second(),
                dateTime._innerTimeSpan.millisecond(),
                dateTime._innerTimeSpan.microsecond());
        os << retVal.data();
        return os;
    }

    auto DateTime::operator<(const DateTime &other) const noexcept -> bool {
        if (this->_innerUTCTimeMicroseconds < other._innerUTCTimeMicroseconds) {
            return true;
        }
        return false;
    }

    auto DateTime::operator>(const DateTime &other) const noexcept -> bool {
        if (this->_innerUTCTimeMicroseconds > other._innerUTCTimeMicroseconds) {
            return true;
        }
        return false;
    }

    auto DateTime::operator==(const DateTime &other) const noexcept -> bool {
        if (this->_innerUTCTimeMicroseconds == other._innerUTCTimeMicroseconds) {
            return true;
        }
        return false;
    }

    auto DateTime::operator<=(const DateTime &other) const noexcept -> bool {
        if (this->_innerUTCTimeMicroseconds <= other._innerUTCTimeMicroseconds) {
            return true;
        }
        return false;
    }

    auto DateTime::operator>=(const DateTime &other) const noexcept -> bool {
        if (this->_innerUTCTimeMicroseconds >= other._innerUTCTimeMicroseconds) {
            return true;
        }
        return false;
    }

    auto DateTime::operator!=(const DateTime &other) const noexcept -> bool {
        if (this->_innerUTCTimeMicroseconds != other._innerUTCTimeMicroseconds) {
            return true;
        }
        return false;
    }

    auto DateTime::operator-(const DateTime &other) const -> TimeSpan {
        uint64_t time;
        if (*this > other) {
            time = this->_innerUTCTimeMicroseconds - other._innerUTCTimeMicroseconds;
        } else {
            time = other._innerUTCTimeMicroseconds - this->_innerUTCTimeMicroseconds;
        }
        return TimeSpan(time);
    }

    auto DateTime::operator+(const TimeSpan &other) -> DateTime {
        auto dateTime = DateTime(*this);
        dateTime._innerUTCTimeMicroseconds += other.total_microseconds();

        dateTime._innerTimeSpan._innerMicroseconds += other.microsecond();
        if (dateTime._innerTimeSpan._innerMicroseconds >= 1000) {
            dateTime._innerTimeSpan._innerMicroseconds -= 1000;
            dateTime._innerTimeSpan._innerMilliseconds += 1;
        }

        dateTime._innerTimeSpan._innerMilliseconds += other.millisecond();
        if (dateTime._innerTimeSpan._innerMilliseconds >= 1000) {
            dateTime._innerTimeSpan._innerMilliseconds -= 1000;
            dateTime._innerTimeSpan._innerSeconds += 1;
        }

        dateTime._innerTimeSpan._innerSeconds += other.second();
        if (dateTime._innerTimeSpan._innerSeconds >= 60) {
            dateTime._innerTimeSpan._innerSeconds -= 60;
            dateTime._innerTimeSpan._innerMinutes += 1;
        }

        dateTime._innerTimeSpan._innerMinutes += other.minute();
        if (dateTime._innerTimeSpan._innerMinutes >= 60) {
            dateTime._innerTimeSpan._innerMinutes -= 60;
            dateTime._innerTimeSpan._innerHours += 1;
        }

        dateTime._innerTimeSpan._innerHours += other.hour();
        if (dateTime._innerTimeSpan._innerHours >= 24) {
            dateTime._innerTimeSpan._innerHours -= 24;
            dateTime._innerTimeSpan._innerDays += 1;
        }

        dateTime._innerTimeSpan._innerDays += other.day();
        uint16_t days = getDaysOfMonth(year(), month());
        while (dateTime._innerTimeSpan._innerDays >= days) {
            dateTime._innerTimeSpan._innerDays -= days;
            dateTime._innerMonth += 1;
            if (dateTime._innerMonth > 12) {
                dateTime._innerMonth = 1;
                dateTime._innerYear += 1;
            }
            days = getDaysOfMonth(year(), month());
        }

        return dateTime;
    }

    auto DateTime::operator+=(const TimeSpan &other) -> DateTime {
        *this = *this + other;
        return *this;
    }

// endregion
    [[nodiscard]]
    [[maybe_unused]]
    auto DateTime::to_string() const noexcept -> std::string {
        std::array<char, 128> retVal{};
        sprintf(retVal.data(),
                "%d-%02d-%02d %02d:%02d:%02d.%03d%03d",
                _innerYear,
                _innerMonth,
                _innerTimeSpan.day(),
                _innerTimeSpan.hour(),
                _innerTimeSpan.minute(),
                _innerTimeSpan.second(),
                _innerTimeSpan.millisecond(),
                _innerTimeSpan.microsecond());
        return {retVal.data()};
    }

    [[maybe_unused]]
    void DateTime::addYears(const int32_t years) {
        _innerYear += years;
        const uint64_t time = getUTCTimeFromLocalTime(_innerYear,
                                                      _innerMonth,
                                                      _innerTimeSpan.day(),
                                                      _innerTimeSpan.hour(),
                                                      _innerTimeSpan.minute(),
                                                      _innerTimeSpan.second(),
                                                      _innerTimeSpan.millisecond(),
                                                      _innerTimeSpan.microsecond());
        initializeTimeFromUTC(time, TimeAccuracy::Microsecond);
    }

    [[maybe_unused]]
    void DateTime::addMonths(const int32_t months) {
        const int years = static_cast<int32_t>(std::floor((static_cast<double>(_innerMonth) + static_cast<double>(months) - 1) / 12));
        _innerYear += years;
        _innerMonth         = (_innerMonth + months - 1) % 12 + 1;
        const uint64_t time = getUTCTimeFromLocalTime(_innerYear,
                                                      _innerMonth,
                                                      _innerTimeSpan.day(),
                                                      _innerTimeSpan.hour(),
                                                      _innerTimeSpan.minute(),
                                                      _innerTimeSpan.second(),
                                                      _innerTimeSpan.millisecond(),
                                                      _innerTimeSpan.microsecond());
        initializeTimeFromUTC(static_cast<int64_t>(time), TimeAccuracy::Microsecond);
    }

    [[maybe_unused]]
    void DateTime::addDays(const int32_t days) {
        addHours(days * 24);
    }

    [[maybe_unused]]
    void DateTime::addHours(const int32_t hours) {
        addMinutes(static_cast<int64_t>(hours) * 60);
    }

    [[maybe_unused]]
    void DateTime::addMinutes(const int64_t minutes) {
        addSeconds(minutes * 60);
    }

    [[maybe_unused]]
    void DateTime::addSeconds(const int64_t seconds) {
        addMilliSeconds(seconds * 1000);
    }

    [[maybe_unused]]
    void DateTime::addMilliSeconds(const int64_t milliseconds) {
        addMicroSeconds(milliseconds * 1000);
    }

    [[maybe_unused]]
    void DateTime::addMicroSeconds(const int64_t microseconds) {
        _innerUTCTimeMicroseconds += microseconds;
        initializeTimeFromUTC(static_cast<int64_t>(_innerUTCTimeMicroseconds), TimeAccuracy::Microsecond);
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto DateTime::isLeapYear() const -> bool {
        return isLeapYear(this->_innerYear);
    }

    [[nodiscard]]
    auto DateTime::isLeapYear(const uint32_t y) -> bool {
        return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
    }
// endregion

// region Get/Set选择器
    [[nodiscard]]
    [[maybe_unused]]
    auto DateTime::year() const noexcept -> int32_t {
        return _innerYear;
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto DateTime::month() const noexcept -> uint32_t {
        return _innerMonth;
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto DateTime::day() const noexcept -> uint32_t {
        return _innerTimeSpan.day();
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto DateTime::hour() const noexcept -> uint32_t {
        return _innerTimeSpan.hour();
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto DateTime::minute() const noexcept -> uint32_t {
        return _innerTimeSpan.minute();
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto DateTime::second() const noexcept -> uint32_t {
        return _innerTimeSpan.second();
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto DateTime::millisecond() const noexcept -> uint32_t {
        return _innerTimeSpan.millisecond();
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto DateTime::microsecond() const noexcept -> uint32_t {
        return _innerTimeSpan.microsecond();
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto DateTime::daysInYear() const noexcept -> uint32_t {
        return _innerDaysInYear;
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto DateTime::daysInWeek() const noexcept -> uint32_t {
        return _innerDaysInWeek;
    }
// endregion

// region 私有方法
    auto DateTime::checkTimeInvalid(const uint32_t year, const uint32_t month, const uint32_t day,
                                    const uint32_t hour, const uint32_t minute, const uint32_t second,
                                    const uint32_t millisecond, const uint32_t microsecond) -> std::string {
        std::array<std::string, 8> resValue = {"year", "Month", "Day", "Hour", "Minute", "Second", "Millisecond",
                                               "Microsecond"};
        const std::array<uint32_t, 8> paraValue    = {year, month, day, hour, minute, second, millisecond, microsecond};
        const std::array<uint32_t, 8> minValue     = {1970, 1, 1, 0, 0, 0, 0, 0};
        constexpr std::array<uint32_t, 8> maxValue = {UINT_MAX, 12, 31, 23, 59, 59, 999, 999};
        for (int i = 0; i < 8; ++i) {
            if (paraValue[i] < minValue[i] || paraValue[i] > maxValue[i]) {
                resValue[i] = resValue[i].append("must between");
                resValue[i] = resValue[i].append(std::to_string(minValue[i]));
                resValue[i] = resValue[i].append("and");
                resValue[i] = resValue[i].append(std::to_string(maxValue[i]));
                return {resValue[i]};
            }
        }
        return {};
    }

    auto DateTime::getUTCTimeFromLocalTime(const int32_t year, const uint32_t month, const uint32_t day,
                                           const uint32_t hour, const uint32_t minute, const uint32_t second,
                                           const uint32_t millisecond, const uint32_t microsecond) -> uint64_t {
        if (const std::string checkRes = checkTimeInvalid(year, month, day, hour, minute, second, millisecond, microsecond); !checkRes.empty()) {
            throw std::invalid_argument(checkRes);
        }
        const auto tmPtr  = new tm();
        tmPtr->tm_year    = static_cast<int32_t>(year) - 1900;
        tmPtr->tm_mon     = static_cast<int32_t>(month) - 1;
        tmPtr->tm_mday    = static_cast<int32_t>(day);
        tmPtr->tm_hour    = static_cast<int32_t>(hour);
        tmPtr->tm_min     = static_cast<int32_t>(minute);
        tmPtr->tm_sec     = static_cast<int32_t>(second);
        tmPtr->tm_isdst   = -1;
        const time_t time = mktime(tmPtr);
        return time * 1000000 + static_cast<uint64_t>(millisecond) * 1000 + microsecond;
    }

    auto DateTime::initializeTimeFromUTC(uint64_t time, const TimeAccuracy timeAccuracy) -> void {
        _innerUTCTimeMicroseconds = time;
        uint16_t microseconds = 0, milliseconds = 0;
        if (timeAccuracy >= TimeAccuracy::Microsecond) {
            microseconds = time % 1000;
            time = time / 1000;
        }
        if (timeAccuracy >= TimeAccuracy::Millisecond) {
            milliseconds = time % 1000;
            time = time / 1000;
        }
        auto tmPtr = tm();
#ifdef _WIN32
        const auto insideTime = static_cast<time_t>(time);
        localtime_s(&tmPtr, &insideTime);
#else
        localtime_r(reinterpret_cast<const time_t *>(&time), &tmPtr);
#endif
        _innerTimeSpan = TimeSpan(tmPtr.tm_mday, tmPtr.tm_hour, tmPtr.tm_min, tmPtr.tm_sec, milliseconds,
                                         microseconds);
        _innerMonth = tmPtr.tm_mon + 1;
        _innerYear = tmPtr.tm_year + 1900;
        _innerDaysInWeek = tmPtr.tm_wday;
        _innerDaysInYear = tmPtr.tm_yday + 1;
    }

    auto DateTime::getDaysOfMonth(const uint16_t year, const uint16_t month) -> uint16_t {
        constexpr std::array<uint16_t, 12> daysOfMonth{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        if (month == 2 && isLeapYear(year)) {
            return 29;
        }
        return daysOfMonth[month - 1];
    }
// endregion
}
#undef PRETTY_FILE_NAME