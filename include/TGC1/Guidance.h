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
#include <deque>
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
        GCInfo getMissionGCInfo(double flyTime, double P, double Mass, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state, double maxLoad,
                                const std::deque<Eigen::Vector3d> &waypoints, int &currentWpIndex);

        /*!
         * @brief 获取制导控制信息
         * @param state 自身状态
         * @param maxLoad 最大过载
         * @param waypoints
         * @param currentWpIndex
         * @return
         */
        GCInfo getGCInfoRouteL1(const State &state, double maxLoad, const std::deque<Eigen::Vector3d> &waypoints, int &currentWpIndex);

        /*!
         * @brief 基于解析阻力加速度剖面的滑翔段中制导
         * @param Mass 当前质量 kg
         * @param targetPosEcf 目标位置
         * @param targetVelEcf 目标速度
         * @param state 当前状态
         * @param maxLoad 最大过载
         * @return 制导控制信息
         */
        GCInfo getGCInfoAnalyticGlide(double Mass, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state, double maxLoad);

        /*!
         * @brief l1制导律调用入口
         * @param maxLoad 最大过载
         * @param currentPosEcf 当前位置
         * @param currentVelEcf 当前速度
         * @param waypoints 航点（经度，纬度，高度）
         * @param currentWpIndex 当前航段起点索引（会被更新）
         * @return 侧向加速度指令   巡航高度
         */
        std::pair<double, double> calculateL1Guidance(double maxLoad, const Eigen::Vector3d &currentPosEcf, const Eigen::Vector3d &currentVelEcf,
                                                      const std::deque<Eigen::Vector3d> &waypoints, int &currentWpIndex);

        /*!
        * @brief 核心L1制导算法 - 修正版
        * @param pos_nue 当前位置[N, E]（忽略天向）
        * @param vel_nue 当前速度[N, E]
        * @param wp_start 航段起点[N, E]
        * @param wp_end 航段终点[N, E]
        * @param L1_distance L1距离
        * @return 横向加速度（m/s²）
        */
        double calculateL1GuidanceNUE(const Eigen::Vector2d &pos_nue, const Eigen::Vector2d &vel_nue, const Eigen::Vector2d &wp_start, const Eigen::Vector2d &wp_end,
                                      double L1_distance = 100.0);

        /*!
         * @brief 兰伯特1
         * @param r_m 起始点ecf坐标
         * @param r_pip 落点ecf坐标
         * @param T_pip 飞行时间 s
         * @param vd_m 发射速度向量ecf
         * @param Range 射程
         */
        void Lambert_Resolve_Dv1(const Eigen::Vector3d &r_m, const Eigen::Vector3d &r_pip, double &T_pip, double vd_m[3], double &Range) const;

        /*!
         * @brief 兰伯特
         * @param r_m 起始点ecf坐标
         * @param r_pip 落点ecf坐标
         * @param T_pip 飞行时间 s
         * @param vd_m 发射速度向量ecf
         * @param Range 射程
         */
        void Lambert_Resolve_Dv(const Eigen::Vector3d &r_m, const Eigen::Vector3d &r_pip, double &T_pip, double vd_m[3], double &Range) const;

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
        double lastDesiredH = 0;
        /*!
         * @brief 侧向翻转逻辑的上一拍符号，避免航向指令在零附近抖振
         */
        double lastBankSign = 1.0;
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

        static double wrapAngle(double angle);

        static double clamp(double value, double minValue, double maxValue);

        static double computeBlendRatio(double targetDistance, double startDistance, double endDistance);

        GCInfo getGCInfoAnalyticMidcourse(double flyTime, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state, double mass, double maxLoad);

        double estimateDragAcceleration(const State &state, double mass) const;

        double estimateLiftAcceleration(const State &state, double mass, double maxLoad) const;

        double solveDragProfileMidpoint(double v0, double vf, double d0, double df, double rangeToGo) const;

        double dragProfile(double v, double v0, double vf, double d0, double dc, double df) const;

        double dragProfileSlope(double v, double v0, double vf, double d0, double dc, double df) const;

        double profileRange(double v0, double vf, double d0, double dc, double df) const;

        static double guidanceHeadingSign(double headingError, double corridor);

// endregion
    };
}
#undef PRETTY_FILE_NAME