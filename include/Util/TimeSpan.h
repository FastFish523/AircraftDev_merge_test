//
// Created by MikuSoft on 2025/11/11.
// Copyright (c) 2025 JiuTianAoXiang All rights reserved.
//

#pragma once 

// region Include
// region STL
#include <cstdint>
// endregion
// region ThirdParty
// endregion
// region Self
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Utils/TimeSpan"
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
class Dll_Export_Import TimeSpan {
// region Using/Friend
    private:
        friend class DateTime;
// endregion
// region 构造函数
    public:
        /*!
         * 时间差
         * @param time 总微秒
         */
        explicit TimeSpan(uint64_t time);

        /*!
         * 时间差
         * @param days 天
         * @param hours 小时
         * @param minutes 分钟
         * @param seconds 秒钟
         * @param milliseconds 毫秒
         * @param microseconds 微秒
         */
        TimeSpan(uint32_t days, uint32_t hours, uint32_t minutes, uint32_t seconds,
                 uint32_t milliseconds, uint32_t microseconds) noexcept;
// endregion

// region 公有属性
    public:
// endregion

// region 公有方法
    public:
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
        auto total_hours() const noexcept -> uint64_t;

        [[nodiscard]]
        [[maybe_unused]]
        auto total_minutes() const noexcept -> uint64_t;

        [[nodiscard]]
        [[maybe_unused]]
        auto total_seconds() const noexcept -> uint64_t;

        [[nodiscard]]
        [[maybe_unused]]
        auto total_milliseconds() const noexcept -> uint64_t;

        [[nodiscard]]
        [[maybe_unused]]
        auto total_microseconds() const noexcept -> uint64_t;
// endregion

// region Get/Set选择器
    public:
// endregion

// region 私有变量
    private:
        uint32_t _innerDays = 0;
        uint32_t _innerHours = 0;
        uint32_t _innerMinutes = 0;
        uint32_t _innerSeconds = 0;
        uint32_t _innerMilliseconds = 0;
        uint32_t _innerMicroseconds = 0;
// endregion

// region 私有方法
    private:
// endregion
};
}
#undef Dll_Export_Import
#undef PRETTY_FILE_NAME