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
#include "Aerodynamics.h"

#include <iostream>

#include "CoordinateHelper.h"
#include "State.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Utils/Aerodynamics"
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

    Eigen::Vector3d Aerodynamics::getF(const State &state, const Eigen::Vector3d &rudder, const double s, const double l, const double b) {
        const double _sref = s;
        const double _lref           = l;
        const double _bref           = b;
        Eigen::Vector3d lla    = Utils::CoordinateHelper::ecefToLla(state.posEcf);
        double alpha, beta;
        Eigen::Vector3d vel_nue = Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        Utils::CoordinateHelper::calculateAngleOfAttack(vel_nue, state.qbn, alpha, beta);
        double ma                     = vel_nue.norm() / 340;
        const double rho              = calculateAtmosphereDensity(lla.z());
        const double dynamic_pressure = 0.5 * rho * vel_nue.squaredNorm();
        double CD, CL, CZ, Cl, Cm, Cn;
        if(computeAeroCoefficientsCB!=nullptr)
            computeAeroCoefficientsCB(alpha, beta, rudder.x(), rudder.y(), rudder.z(), ma, CD, CL, CZ, Cl, Cm, Cn);
        else
            std::cout<<"computeAeroCoefficientsCB is not set"<<std::endl;
        // 气动力（速度坐标系）
        Eigen::Vector3d force_vel;
        force_vel[0] = dynamic_pressure * _sref * CD; // 阻力
        force_vel[1] = dynamic_pressure * _sref * CL; // 升力
        force_vel[2] = dynamic_pressure * _sref * CZ; // 侧向力
        // 转换到体坐标系
        const Eigen::Vector3d force_body = Utils::CoordinateHelper::velocityToBodyAcceleration(force_vel, alpha, beta);
        // 转换到NUE系
        const Eigen::Vector3d force_nue = Utils::CoordinateHelper::bodyToNueAcceleration(force_body, state.qbn);
        // 转换到ECF系
        return Utils::CoordinateHelper::nueToEcefAcceleration(force_nue, lla.x(), lla.y());
    }

    Eigen::Vector3d Aerodynamics::getM(const State &state, const Eigen::Vector3d &rudder, const double s, const double l, const double b) {
        const double _sref = s;
        const double _lref = l;
        const double _bref = b;
        Eigen::Vector3d lla    = Utils::CoordinateHelper::ecefToLla(state.posEcf);
        double alpha, beta;
        Eigen::Vector3d vel_nue = Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        Utils::CoordinateHelper::calculateAngleOfAttack(vel_nue, state.qbn, alpha, beta);
        double ma                     = vel_nue.norm() / 340;
        const double rho              = calculateAtmosphereDensity(lla.z());
        const double dynamic_pressure = 0.5 * rho * vel_nue.squaredNorm();
        double CD, CL, CZ, Cl, Cm, Cn;
        if(computeAeroCoefficientsCB!=nullptr)
            computeAeroCoefficientsCB(alpha, beta, rudder.x(), rudder.y(), rudder.z(), ma, CD, CL, CZ, Cl, Cm, Cn);
        else
            std::cout<<"computeAeroCoefficientsCB is not set"<<std::endl;
        Eigen::Vector3d moment_body;
        moment_body[0] = dynamic_pressure * _sref * _bref * Cl;
        moment_body[1] = dynamic_pressure * _sref * _lref * Cn;
        moment_body[2] = dynamic_pressure * _sref * _lref * Cm;
        return moment_body;
    }

// endregion

// region Get/Set选择器
// endregion

// region Private Methods
    double Aerodynamics::calculateAtmosphereDensity(const double altitude) {
        const double h            = altitude / 1000.0;  // 高度 [km]
        Environment::Utils::Atmosphere _atmosphere;
        return _atmosphere.getDensity(2000,1,0,0,0,h);
    }
// endregion
}
#undef PRETTY_FILE_NAME
