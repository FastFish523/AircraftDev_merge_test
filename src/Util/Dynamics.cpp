//
// Created by MikuSoft on 2025/11/11.
// Copyright (c) 2025 JiuTianAoXiang All rights reserved.
//
// region Include
// region STL
// endregion
// region ThirdParty
#include "Constants.h"
// endregion
// region Self
#include "Dynamics.h"
#include "Aerodynamics.h"
#include "CoordinateHelper.h"

// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Utils/Dynamics"
// endregion

// region Using NameSpace
// endregion

namespace ModelDevelop::Utils {
// region Static Attributes Init
// endregion

// region USING/FRIEND
// endregion

// region Constructor
// endregion

    Eigen::Vector3d Dynamics::Acc_Ecf(const State& state, const double mass,const Eigen::Matrix3d& inertia,const Eigen::Vector3d& P_body,const Eigen::Vector3d& rudder, const double s, const double l, const double b) {
        Eigen::Vector3d lla = CoordinateHelper::ecefToLla(state.posEcf);
        // 引力加速度（简化中心引力场）
        const auto gravity_accel = CoordinateHelper::calculateGravity(state.posEcf);
        // 转换气动力和控制力到ECF系
        const auto aero_force_ecf   = _aerodynamics.getF(state,rudder,s,l,b);
        const auto P_nue = CoordinateHelper::bodyToNueAcceleration(P_body,state.qbn);
        const auto control_force_ecf = CoordinateHelper::nueToEcefAcceleration(P_nue,lla.x(),lla.y());

        // 科里奥利加速度和离心加速度（由于地球自转）
        const Eigen::Vector3d omega_earth(0, 0, Constants::EARTH_OMEGA);
        const auto coriolis_accel    = (-2.0 * omega_earth.cross(state.velEcf)).eval();
        const auto centrifugal_accel = (-omega_earth.cross(omega_earth.cross(state.posEcf))).eval();
        // 总加速度
        auto acceleration_ecf = (gravity_accel + (aero_force_ecf + control_force_ecf) / mass + coriolis_accel + centrifugal_accel).eval();
        return acceleration_ecf;

    }

    Eigen::Vector3d Dynamics::Moment_Body(const State& state,double mass,const Eigen::Matrix3d& inertia,const Eigen::Vector3d& M_body,const Eigen::Vector3d& rudder, const double s, const double l, const double b) {
        const Eigen::Vector3d aero_m = _aerodynamics.getM(state,rudder,s,l,b);
        const Eigen::Vector3d& direct_m = M_body;

        return aero_m + direct_m;
    }

// region Public Methods

// endregion

// region Get/Set选择器

// endregion

// region Private Methods
// endregion
}
#undef PRETTY_FILE_NAME
