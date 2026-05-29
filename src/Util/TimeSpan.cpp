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
#include "TimeSpan.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Utils/TimeSpan"
// endregion

// region Using NameSpace
// endregion

namespace ModelDevelop::Utils {
// region Static Attributes Init
// endregion

// region USING/FRIEND
// endregion

// region 构造函数
    TimeSpan::TimeSpan(uint64_t time) {
        this->_innerMicroseconds = time % 1000;
        time = time / 1000;
        this->_innerMilliseconds = time % 1000;
        time = time / 1000;
        this->_innerSeconds = time % 60;
        time = time / 60;
        this->_innerMinutes = time % 60;
        time = time / 60;
        this->_innerHours = time % 24;
        time = time / 24;
        this->_innerDays = (uint32_t) time;
    }

/*!
 * 时间差
 * @param days 天
 * @param hours 小时
 * @param minutes 分钟
 * @param seconds 秒钟
 * @param milliseconds 毫秒
 * @param microseconds 微秒
 */
    TimeSpan::TimeSpan(uint32_t days, uint32_t hours,
                       uint32_t minutes, uint32_t seconds,
                       uint32_t milliseconds, uint32_t microseconds) noexcept {
        _innerMicroseconds = microseconds % 1000;
        milliseconds += (microseconds - _innerMicroseconds) / 1000;
        _innerMilliseconds = milliseconds % 1000;
        seconds += (milliseconds - _innerMilliseconds) / 1000;
        _innerSeconds = seconds % 60;
        minutes += (seconds - _innerSeconds) / 60;
        _innerMinutes = minutes % 60;
        hours += (minutes - _innerMinutes) / 60;
        _innerHours = hours % 24;
        _innerDays = days + (hours - _innerHours) / 24;
    }
// endregion

// region 公有方法
    [[nodiscard]]
    [[maybe_unused]]
    auto TimeSpan::day() const noexcept -> uint32_t {
        return _innerDays;
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto TimeSpan::hour() const noexcept -> uint32_t {
        return _innerHours;
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto TimeSpan::minute() const noexcept -> uint32_t {
        return _innerMinutes;
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto TimeSpan::second() const noexcept -> uint32_t {
        return _innerSeconds;
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto TimeSpan::millisecond() const noexcept -> uint32_t {
        return _innerMilliseconds;
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto TimeSpan::microsecond() const noexcept -> uint32_t {
        return _innerMicroseconds;
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto TimeSpan::total_hours() const noexcept -> uint64_t {
        return _innerDays * 24 + _innerHours;
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto TimeSpan::total_minutes() const noexcept -> uint64_t {
        return total_hours() * 60 + _innerMinutes;
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto TimeSpan::total_seconds() const noexcept -> uint64_t {
        return total_minutes() * 60 + _innerSeconds;
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto TimeSpan::total_milliseconds() const noexcept -> uint64_t {
        return total_seconds() * 1000 + _innerMilliseconds;
    }

    [[nodiscard]]
    [[maybe_unused]]
    auto TimeSpan::total_microseconds() const noexcept -> uint64_t {
        return total_milliseconds() * 1000 + _innerMicroseconds;
    }
// endregion
// region Get/Set选择器
// endregion

// region Private Methods
// endregion
}
#undef PRETTY_FILE_NAME