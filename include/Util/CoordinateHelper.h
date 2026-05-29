//
// Created by MikuSoft on 2025/11/11.
// Copyright (c) 2025 JiuTianAoXiang All rights reserved.
//

#pragma once

// region Include
// region STL
// endregion
// region ThirdParty
#include <iostream>

#include "CJulian.h"
#include "Constants.h"
#include "Eigen/Core"
#include "Eigen/Dense"
// endregion
// region Self
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Utils/CoordinateHelper"
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
    class Dll_Export_Import CoordinateHelper {
    public:
        CoordinateHelper() = default;

        ~CoordinateHelper() = default;

        /*!
         * @brief 计算引力加速度(ecf)
         * @param position_ecf
         * @return
         */
        static Eigen::Vector3d calculateGravity(const Eigen::Vector3d &position_ecf);

        // region 纯向量转换，只在数学上有意义
        /*!
         * @brief NUE向量到ECEF向量转换
         * @param nue_vec
         * @param _lon
         * @param _lat
         * @return
         */
        static Eigen::Vector3d nueToEcefVector(const Eigen::Vector3d &nue_vec, const double _lon, const double _lat) ;

        /*!
         * @brief ECEF向量到NUE向量转换
         * @param ecef_vec
         * @param _lon
         * @param _lat
         * @return
         */
        static Eigen::Vector3d ecefToNueVector(const Eigen::Vector3d &ecef_vec, const double _lon, const double _lat) ;

        /*!
         * @brief ECF向量到ECI向量转换
         * @param ecef_vec
         * @param dt
         * @return
         */
        static Eigen::Vector3d ecefToEciVector(const Eigen::Vector3d &ecef_vec, const DateTime &dt) ;

        /*!
         * @brief ECI向量到ECF向量转换
         * @param eci_vec
         * @param dt
         * @return
         */
            static Eigen::Vector3d eciToEcefVector(const Eigen::Vector3d &eci_vec, const DateTime &dt) ;

        /*!
         * @brief 体坐标系向量到NUE向量转换
         * @param body_vec
         * @param qbn
         * @return
         */
        static Eigen::Vector3d bodyToNueVector(const Eigen::Vector3d &body_vec, const Eigen::Quaterniond &qbn) ;

        /*!
         * @brief NUE向量到体坐标系向量转换
         * @param nue_vec
         * @param qbn
         * @return
         */
        static Eigen::Vector3d nueToBodyVector(const Eigen::Vector3d &nue_vec, const Eigen::Quaterniond &qbn) ;

        // 速度坐标系到体坐标系转换
        static Eigen::Vector3d velocityToBodyVector(const Eigen::Vector3d &vec_vel, const double alpha, const double beta) ;

        // 体系到速度系转换
        static Eigen::Vector3d bodyToVelocityVector(const Eigen::Vector3d &vec_body, const double alpha, const double beta) ;

        // endregion

        // region 计算位置
        /*!
         * @brief 基于ECEF计算经纬高（WGS84）
         * @param ecef_pos
         * @return
         */
        static Eigen::Vector3d ecefToLla(const Eigen::Vector3d &ecef_pos) ;

        /*!
         * @brief 基于经纬高计算ECEF位置
         * @param lla
         * @return
         */
        static Eigen::Vector3d llaToEcef(const Eigen::Vector3d &lla);

        /*!
         * @brief 基于ECI计算ECF位置转换（需要格林尼治恒星时）
         * @param eci_pos
         * @param dt
         * @return
         */
        static Eigen::Vector3d eciToEcefPosition(const Eigen::Vector3d &eci_pos, const DateTime &dt) ;

        /*!
         * @brief 基于ECF计算ECI位置转换
         * @param ecef_pos
         * @param dt
         * @return
         */
        static Eigen::Vector3d ecefToEciPosition(const Eigen::Vector3d &ecef_pos, const DateTime &dt);

        /*!
         * @brief 基于NUE位置计算ECEF位置
         * @param nue_pos
         * @param lon
         * @param lat
         * @return
         */
        static Eigen::Vector3d nueToEcefPosition(const Eigen::Vector3d &nue_pos, double lon, double lat) ;

        /*!
         * @brief 基于ECEF计算NUE位置
         * @param ecef_pos
         * @param lon
         * @param lat
         * @return
         */
        static Eigen::Vector3d ecefToNuePosition(const Eigen::Vector3d &ecef_pos, const double lon, const double lat) ;

        // endregion

        // region 计算速度
        /*!
         * @brief 基于ECF速度计算ECI速度
         * @param ecef_vel
         * @param ecef_pos
         * @param dt
         * @return
         */
        static Eigen::Vector3d ecefToEciVelocity(const Eigen::Vector3d &ecef_vel, const Eigen::Vector3d &ecef_pos, const DateTime &dt);

        /*!
         * @brief 基于ECI速度计算ECF速度
         * @param eci_vel
         * @param eci_pos
         * @param dt
         * @return
         */
        static Eigen::Vector3d eciToEcefVelocity(const Eigen::Vector3d &eci_vel, const Eigen::Vector3d &eci_pos, const DateTime &dt);

        /*!
         * @brief NUE速度到ECEF速度转换（ECF系，不考虑地球自转）
         * @param nue_vel
         * @param lon
         * @param lat
         * @return
         */
        static Eigen::Vector3d nueToEcefVelocity(const Eigen::Vector3d &nue_vel, const double lon, const double lat);

        /*!
         * @brief ECEF速度到NUE速度转换（ECF系）
         * @param ecef_vel
         * @param lon
         * @param lat
         * @return
         */
        static Eigen::Vector3d ecefToNueVelocity(const Eigen::Vector3d &ecef_vel, const double lon, const double lat) ;


        /*!
         * @brief 基于体坐标系速度计算NUE速度
         * @param body_vel
         * @param qbn
         * @return
         */
        static Eigen::Vector3d bodyToNueVelocity(const Eigen::Vector3d &body_vel, const Eigen::Quaterniond &qbn) ;

        /*!
         * @brief 基于NUE速度计算体坐标系速度
         * @param nue_vel
         * @param qbn
         * @return
         */
        static Eigen::Vector3d nueToBodyVelocity(const Eigen::Vector3d &nue_vel, const Eigen::Quaterniond &qbn) ;

        // endregion

        // region 计算姿态
        /*!
         * @brief 姿态转四元数
         * @param yaw
         * @param pitch
         * @param roll
         * @return
         */
        static Eigen::Quaterniond euler231ToQuaternion(double yaw, double pitch, double roll) ;

        /*!
         * @brief 四元数转姿态
         * @param qbn
         * @return
         */
        static Eigen::Vector3d quaternionToEuler231(const Eigen::Quaterniond &qbn) ;

        /*!
         * @brief 计算攻角和侧滑角
         * @param velocity_nue
         * @param attitude
         * @param alpha
         * @param beta
         */
        static void calculateAngleOfAttack(const Eigen::Vector3d &velocity_nue, const Eigen::Quaterniond &attitude, double &alpha, double &beta) ;

        // endregion

        // region 计算加速度
        /*!
         * @brief NUE加速度到ECEF加速度转换（ECF系）
         * @param nue_accel
         * @param lon
         * @param lat
         * @return
         */
        static Eigen::Vector3d nueToEcefAcceleration(const Eigen::Vector3d &nue_accel, const double lon, const double lat);

        /*!
         * @brief ECEF加速度到NUE加速度转换（ECF系）
         * @param ecef_accel
         * @param lon
         * @param lat
         * @return
         */
        static Eigen::Vector3d ecefToNueAcceleration(const Eigen::Vector3d &ecef_accel, const double lon, const double lat);

        /*!
         * @brief 体坐标系加速度到NUE加速度转换
         * @param body_accel
         * @param qbn
         * @return
         */
        static Eigen::Vector3d bodyToNueAcceleration(const Eigen::Vector3d &body_accel, const Eigen::Quaterniond &qbn);

        /*!
         * @brief NUE加速度到体坐标系加速度转换
         * @param nue_accel
         * @param qbn
         * @return
         */
        static Eigen::Vector3d nueToBodyAcceleration(const Eigen::Vector3d &nue_accel, const Eigen::Quaterniond &qbn);

        static Eigen::Vector3d velocityToBodyAcceleration(const Eigen::Vector3d &vel_accel, const double alpha, const double beta) ;

        static Eigen::Vector3d bodyToVelocityAcceleration(const Eigen::Vector3d &body_accel, const double alpha, const double beta);

        static Eigen::Vector3d velocityToNue(const Eigen::Vector3d& vel_data, const Eigen::Vector3d& vel_nue) ;

        static Eigen::Vector3d nueToVelocity(const Eigen::Vector3d& nue_data, const Eigen::Vector3d& vel_nue) ;

        /*!
         * @brief 计算倾角
         * @param vec_nue
         * @return
         */
        static double getTheta(Eigen::Vector3d vec_nue) ;

        /*!
         * @brief 计算偏角
         * @param vec_nue
         * @return
         */
        static double getPsi(Eigen::Vector3d vec_nue) ;

        // endregion
    };
}
#undef Dll_Export_Import
#undef PRETTY_FILE_NAME
