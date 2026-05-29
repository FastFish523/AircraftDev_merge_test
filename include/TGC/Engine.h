//
// Created by 17298 on 2026/4/22.
//


#pragma once

// region Include
// region STL
#include <array>
// endregion
// region ThirdParty
// endregion
// region Self
#include "CommonStructs.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "State.h"
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
#if defined(_WIN32) && !defined(StaticTGC_Build)
#ifdef SharedTGC_Build
#define TGC_ENGINE_API __declspec(dllexport)
#else
#define TGC_ENGINE_API __declspec(dllimport)
#endif
#else
#define TGC_ENGINE_API
#endif
// endregion

namespace ModelDevelop::TGC {
    class TGC_ENGINE_API Engine {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
        Engine() = default;

        ~Engine() = default;

// endregion

// region Public Attributes
    public:
// endregion

// region Public Methods
    public:
        /*!
         * @brief 获取当前助推器推力、剩余安装助推器质量和惯量。
         * @param simStep 仿真步长，秒。
         * @param flyTime 飞行时间，秒。
         * @param state 导弹状态。
         * @param dx 喷管偏转角，弧度。
         * @param dy 喷管偏转角，弧度。
         * @param dz 喷管偏转角，弧度。
         * @return 发动机质量/推力/惯量信息。
         */
        EigenInfo getEigenInfo(double simStep, double flyTime, const State &state, double dx = 0, double dy = 0, double dz = 0);

        void reset();

        [[nodiscard]]
        static double boostTotalTime();

        [[nodiscard]]
        static std::array<double, 3> boostBurnOutTimes();

// endregion

// region Get/Set
    public:
// endregion

// region Private Attributes
    private:
        struct BoostStage {
            double propellantMass;     // kg
            double totalMass;          // kg
            double thrust;             // N
            double burnTime;           // s
        };

        inline static constexpr std::array<BoostStage, 3> boostStages{{
            {45370.0, 48990.0, 2049.6 * 1000.0, 56.4},
            {24490.0, 27670.0, 1222.8 * 1000.0, 60.7},
            {7070.0, 7710.0, 289.1 * 1000.0, 72.0},
        }};

        /*!
         * @brief 附加转动惯量（绕 X 轴）。
         */
        double jx = 0;
        /*!
         * @brief 附加转动惯量（绕 Y 轴）。
         */
        double jy = 0;
        /*!
         * @brief 附加转动惯量（绕 Z 轴）。
         */
        double jz = 0;

// endregion

// region Private Methods
    private:
// endregion
    };
}
#undef TGC_ENGINE_API
#undef PRETTY_FILE_NAME
