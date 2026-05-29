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
#include "Kinematics.h"
#include "Dynamics.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Utils/Kinematics"
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

// region Public Methods
    D_State Kinematics::cal_d_state(const State& state, const double mass,const Eigen::Matrix3d& inertia, const Eigen::Vector3d& P_body,const Eigen::Vector3d& M_body,const Eigen::Vector3d& rudder,const double s,const double l,const double b) {
        D_State d_state;
        //pos
        d_state.d_posEcf = state.velEcf;
        //vel
        d_state.d_velEcf = _dynamics.Acc_Ecf(state,mass,inertia,P_body,rudder,s,l,b);
        //qbn
        const Eigen::Quaterniond q_omega(0, state.wnb_b.x(), state.wnb_b.y(), state.wnb_b.z());
        Eigen::Quaterniond q_dot = state.qbn * q_omega;
        q_dot.coeffs() *= 0.5;  // 乘以0.5
        d_state.d_qbn = q_dot;
        //wnbb
        const auto total_moment = _dynamics.Moment_Body(state,mass,inertia,M_body,rudder,s,l,b);
        const auto inertia_term = state.wnb_b.cross(inertia * state.wnb_b).eval();
        const auto angular_accel_body = (inertia.inverse() * (total_moment - inertia_term)).eval();
        d_state.d_wnb_b = angular_accel_body;

        return d_state;
    }

// endregion

// region Get/Set选择器
// endregion

// region Private Methods

// endregion
}
#undef PRETTY_FILE_NAME
