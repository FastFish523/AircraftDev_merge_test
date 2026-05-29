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
#include "Eigen/Dense"
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
// endregion

namespace ModelDevelop::TGC {
    enum class GuidancePhase {
        Boost = 0,
        Climb,
        Glide,
        Handover,
        Terminal
    };

    struct EigenInfo {
        double mass = 0;
        Eigen::Vector3d P_body{};
        Eigen::Matrix3d inertia;
    };

    struct LosInfo {
        double sigma_elv;
        double sigma_az;
        double sigma_elv_dot;
        double sigma_az_dot;
        double dis_dot;
        double sigma_elv_b;
        double sigma_az_b;
    };

    struct GCInfo {
        LosInfo losInfo{};
        Eigen::Vector3d acc_cmd_v;
        GuidancePhase phase = GuidancePhase::Boost;
        double handoverRatio = 0.0;
    };

    struct ImuInfo {
        Eigen::Vector3d imu_acc_body{0, 0, 0};
        Eigen::Vector3d imu_w_xyz_body{0, 0, 0};
        Eigen::Vector3d imu_ypr{0, 0, 0};
    };

    struct Derivative {
        double a11 = 0;
        double a12 = 0;
        double a13 = 0;
        double a14 = 0;
        double a15 = 0;
        double a16 = 0;

        double a21 = 0;
        double a22 = 0;
        double a23 = 0;
        double a24 = 0;
        double a25 = 0;
        double a26 = 0;

        double a31 = 0;
        double a32 = 0;
        double a33 = 0;
        double a34 = 0;
        double a35 = 0;
        double a36 = 0;

        double b11 = 0;
        double b12 = 0;
        double b13 = 0;
        double b14 = 0;
        double b15 = 0;
        double b16 = 0;
        double b17 = 0;
        double b18 = 0;

        double b21 = 0;
        double b22 = 0;
        double b23 = 0;
        double b24 = 0;
        double b25 = 0;
        double b26 = 0;
        double b27 = 0;
        double b28 = 0;

        double b31 = 0;
        double b32 = 0;
        double b33 = 0;
        double b34 = 0;
        double b35 = 0;
        double b36 = 0;
        double b37 = 0;
        double b38 = 0;
    };
};
#undef PRETTY_FILE_NAME
