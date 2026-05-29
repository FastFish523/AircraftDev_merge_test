//
// Created by MikuSoft on 2025/11/11.
// Copyright (c) 2025 JiuTianAoXiang All rights reserved.
//

#pragma once 

// region Include
// region STL
#include "ostream"
#include "TimeAccuracy.h"
#include "TimeSpan.h"
// endregion
// region ThirdParty
// endregion
// region Self
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Utils/DateTime"
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
class Dll_Export_Import DateTime {
// region 构造函数
    public:
        /*!
         * 获取当前时间
         */
        DateTime();

        /*!
         * 从UTC时间转换为当前系统时区的DateTime
         * @param time UTC时间
         * @param timeAccuracy 时间精度
         */
        explicit DateTime(double time, TimeAccuracy timeAccuracy);

        /*!
         * 从UTC时间转换为当前系统时区的DateTime
         * @param time UTC时间
         * @param timeAccuracy 时间精度
         */
        explicit DateTime(uint64_t time, TimeAccuracy timeAccuracy);

        /*!
         * 获取时间
         * @param year 年
         * @param month 月(1-12)
         * @param day 日(1-28/29/30/31)
         * @param hour 时(0-23)
         * @param minute 分(0-59)
         * @param second 秒(0-59)
         * @param millisecond 毫秒(0-999)
         * @param microsecond 微秒(0-999)
         */
        DateTime(int32_t year, uint32_t month, uint32_t day,
                 uint32_t hour, uint32_t minute, uint32_t second,
                 uint32_t millisecond = 0, uint32_t microsecond = 0);

        ~DateTime() = default;
// endregion

// region 公有属性
    public:
// endregion

// region 公有方法
    public:
// region operator
        friend auto operator<<(std::ostream &os, const DateTime &dateTime) -> std::ostream &;

        auto operator<(const DateTime &other) const noexcept -> bool;

        auto operator>(const DateTime &other) const noexcept -> bool;

        auto operator==(const DateTime &other) const noexcept -> bool;

        auto operator<=(const DateTime &other) const noexcept -> bool;

        auto operator>=(const DateTime &other) const noexcept -> bool;

        auto operator!=(const DateTime &other) const noexcept -> bool;

        auto operator-(const DateTime &other) const -> TimeSpan;

        auto operator+(const TimeSpan &other) -> DateTime;

        auto operator+=(const TimeSpan &other) -> DateTime;

// endregion
        /*!
         * 转换为时间
         * @return
         */
        [[nodiscard]]
        [[maybe_unused]]
        auto to_string() const noexcept -> std::string;

        /*!
         * 获取当前UTC时间
         * @tparam T 类型
         * @param timeAccuracy 时间单位
         * @return
         */
        template<typename T, std::enable_if_t<std::is_same_v<T, uint64_t>
                                              || std::is_same_v<T, double>, int32_t> = 0>
        [[nodiscard]]
        [[maybe_unused]]
        auto toUTCTime(TimeAccuracy timeAccuracy = TimeAccuracy::Millisecond) const noexcept -> T {
            T value = (T) _innerUTCTimeMicroseconds;
            if (timeAccuracy == TimeAccuracy::Microsecond) {
                return value;
            } else if (timeAccuracy == TimeAccuracy::Millisecond) {
                return value / 1000;
            } else {
                return value / (1000 * 1000);
            }
        }

        [[maybe_unused]]
        auto addYears(int32_t years) -> void;

        [[maybe_unused]]
        auto addMonths(int32_t months) -> void;

        [[maybe_unused]]
        auto addDays(int32_t days) -> void;

        [[maybe_unused]]
        auto addHours(int32_t hours) -> void;

        [[maybe_unused]]
        auto addMinutes(int64_t minutes) -> void;

        [[maybe_unused]]
        auto addSeconds(int64_t seconds) -> void;

        [[maybe_unused]]
        auto addMilliSeconds(int64_t milliseconds) -> void;

        [[maybe_unused]]
        auto addMicroSeconds(int64_t microseconds) -> void;

        [[nodiscard]]
        [[maybe_unused]]
        auto isLeapYear() const -> bool;

        [[nodiscard]]
        static auto isLeapYear(uint32_t y) -> bool;
// endregion

// region Get/Set选择器
    public:
        [[nodiscard]]
        [[maybe_unused]]
        auto year() const noexcept -> int32_t;

        [[nodiscard]]
        [[maybe_unused]]
        auto month() const noexcept -> uint32_t;

        [[nodiscard]]
        [[maybe_unused]]
        auto day() const noexcept -> uint32_t;

        [[nodiscard]]
        [[maybe_unused]]
        auto hour() const noexcept -> uint32_t;

        [[nodiscard]]
        [[maybe_unused]]
        auto minute() const noexcept -> uint32_t;

        [[nodiscard]]
        [[maybe_unused]]
        auto second() const noexcept -> uint32_t;

        [[nodiscard]]
        [[maybe_unused]]
        auto millisecond() const noexcept -> uint32_t;

        [[nodiscard]]
        [[maybe_unused]]
        auto microsecond() const noexcept -> uint32_t;

        [[nodiscard]]
        [[maybe_unused]]
        auto daysInYear() const noexcept -> uint32_t;

        [[nodiscard]]
        [[maybe_unused]]
        auto daysInWeek() const noexcept -> uint32_t;
// endregion

// region 私有变量
    private:
        /*!
         * 年
         */
        int32_t _innerYear = 0;
        /*!
         * 月
         */
        uint32_t _innerMonth = 0;
        /*!
         * UTC时间
         */
        uint64_t _innerUTCTimeMicroseconds = 0;
        /*!
         * 日/时/分/秒/毫秒/微秒
         */
        TimeSpan _innerTimeSpan = TimeSpan(0);
        /*!
         * 一年中的第几天
         */
        uint16_t _innerDaysInYear = 0;
        /*!
         * 一周中的第几天
         */
        uint16_t _innerDaysInWeek = 0;
// endregion

// region 私有方法
    private:
        /*!
         * 检查时间是否合法
         * @param year 年
         * @param month 月
         * @param day 日
         * @param hour 时
         * @param minute 分
         * @param second 秒
         * @param millisecond 毫秒
         * @param microsecond 微秒
         * @return
         */
        static auto checkTimeInvalid(uint32_t year, uint32_t month, uint32_t day,
                                     uint32_t hour, uint32_t minute, uint32_t second,
                                     uint32_t millisecond, uint32_t microsecond) -> std::string;

        /*!
         * 从当前系统时间转换到协调世界时
         * @param year 年
         * @param month 月
         * @param day 日
         * @param hour 时
         * @param minute 分
         * @param second 秒
         * @param millisecond 毫秒
         * @param microsecond 微秒
         * @return
         */
        static auto getUTCTimeFromLocalTime(int32_t year, uint32_t month, uint32_t day,
                                            uint32_t hour, uint32_t minute, uint32_t second,
                                            uint32_t millisecond, uint32_t microsecond) -> uint64_t;

        /*!
         * 从协调世界时初始化到当前系统时区的时间
         * @param time UTC时间
         * @param timeAccuracy 时间单位
         */
        auto initializeTimeFromUTC(uint64_t time, TimeAccuracy timeAccuracy) -> void;

        /*!
         * 获取某一个月份的天数
         * @param year 年份
         * @param month 月份
         * @return
         */
        static auto getDaysOfMonth(uint16_t year, uint16_t month) -> uint16_t;
// endregion
};
}
#undef Dll_Export_Import
#undef PRETTY_FILE_NAME