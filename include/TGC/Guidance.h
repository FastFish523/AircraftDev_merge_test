//
// Created by 17298 on 2026/4/22.
//


#pragma once

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "CommonStructs.h"
#include <Eigen/Core>
#include "Seeker.h"
#include "State.h"
#include <optional>
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
// endregion

namespace ModelDevelop::TGC {
    class Guidance {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
        Guidance() = default;

        ~Guidance() = default;

// endregion

// region Public Attributes
    public:
// endregion

// region Public Methods
    public:
        /*!
         * @brief 获取制导控制信息
         * @param flyTime 飞行时间
         * @param P 推力牛
         * @param Mass 质量 千克
         * @param targetPosEcf 目标位置
         * @param targetVelEcf 目标速度
         * @param state 自身状态
         * @param maxLoad 最大过载
         * @return
         */
        GCInfo getMissionGCInfo(double flyTime, double P, double Mass, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state, double maxLoad);

        GCInfo getDiveGCInfo(double flyTime, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state, double maxLoad,
                             const DiveGuidanceConfig &config = DiveGuidanceConfig{});

        void reset();
// endregion

// region Get/Set选择器
    public:
// endregion

// region Private Attributes
    private:
        /*!
         * @brief 导引头
         */
        Seeker _seeker{};
        /*!
         * @brief 上一个预期高度
         */
        double lastBankSign = 1.0;
        std::optional<double> boostInitialPsi = std::nullopt;
        std::optional<double> _lastDiveThetaCmd = std::nullopt;
        std::optional<double> _lastDiveThetaCmdTime = std::nullopt;
        std::optional<GuidancePhase> _lastDivePhase = std::nullopt;
        std::optional<Eigen::Vector3d> _lastTerminalAccCmd = std::nullopt;
        std::optional<double> _lastTerminalAccCmdTime = std::nullopt;
        /*!
         * @brief 兰伯特制导参数
         */
        const double c_dPi = 3.1415926535897932384626433832795;
        /*!
         * @brief 兰伯特制导参数
         */
        const double D2R = c_dPi / 180.0;
        /*!
         * @brief 兰伯特制导参数
         */
        const double R2D = 180.0 / c_dPi;
        /*!
         * @brief 兰伯特制导参数
         */
        const double av = 340.0;
        /*!
         * @brief 兰伯特制导参数
         */
        const double earth_ae = 6371004.0;
        /*!
         * @brief 兰伯特制导参数
         */
        const double earth_g0 = 9.80665;
        /*!
         * @brief 兰伯特制导参数
         */
        const double earth_omega = 7.292115E-5;
        /*!
         * @brief 兰伯特制导参数
         */
        const double c_dMiu = 3.986004418e14;
// endregion

// region Private Methods
    private:
        struct BoostGuidanceInfo {
            Eigen::Vector3d acc_cmd_v{0, 0, 0};
            Eigen::Vector3d tvc_cmd{0, 0, 0};
            double pitch_cmd = 0.0;
            bool pitch_cmd_valid = false;
        };

        /*!
         * 获取视线角速率信息
         * @param targetPosEcf
         * @param targetVelEcf
         * @param state
         * @return
         */
        static LosInfo getLOSInfo(const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state);

        /*!
         * @brief 比例导引法
         * @param theta 速度倾角 弧度
         * @param sigma_az_dot 视线偏角变化率 弧度/秒
         * @param sigma_elv_dot 视线倾角变化率 弧度/秒
         * @param dis_dot 弹目距离变化率 m/s
         * @return
         */
        static Eigen::Vector3d guidance_pn(double theta, double sigma_az_dot, double sigma_elv_dot, double dis_dot);

        BoostGuidanceInfo calculateBoostGuidance(double flyTime, double P, double Mass, const Eigen::Vector3d &targetPosEcf, double maxLoad,
                                                 const State &state);

        BoostGuidanceInfo calculateGlideGuidance(double flyTime, double P, double Mass, const Eigen::Vector3d &targetPosEcf, double maxLoad,
                                                 const State &state);

        BoostGuidanceInfo calculateDiveGuidance(double flyTime, double P, double Mass, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf,
                                                double maxLoad, const State &state);

        static double smoothStep(double ratio);

        static double computeBlendRatio(double targetDistance, double startDistance, double endDistance);

        GCInfo getGCInfoAnalyticMidcourse(double flyTime, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state, double mass,
                                          double maxLoad);

        double estimateDragAcceleration(const State &state, double mass) const;

        double estimateLiftAcceleration(const State &state, double mass, double maxLoad) const;

        double solveDragProfileMidpoint(double v0, double vf, double d0, double df, double rangeToGo) const;

        double dragProfile(double v, double v0, double vf, double d0, double dc, double df) const;

        double dragProfileSlope(double v, double v0, double vf, double d0, double dc, double df) const;

        double profileRange(double v0, double vf, double d0, double dc, double df) const;

        static double guidanceHeadingSign(double headingError, double corridor);

    public:
        static double wrapAngle(double angle);

        static double clamp(double value, double minValue, double maxValue);

// endregion
    };
}
#undef PRETTY_FILE_NAME
