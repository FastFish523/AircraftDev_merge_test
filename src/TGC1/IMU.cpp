//
// Created by 17298 on 2026/4/22.
//

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "TGC/IMU.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
// endregion

// region Using NameSpace

// endregion

namespace ModelDevelop::TGC {
// region Static Attributes Init
// endregion

// region USING/FRIEND
// endregion

// region Constructor
// endregion

// region Public Methods
    ImuInfo IMU::getImuInfoBody(const State &state, const Eigen::Vector3d &total_acc_ecf) {
        ImuInfo imu_info;
        Eigen::Vector3d lla                 = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const Eigen::Vector3d gravity_accel = ModelDevelop::Utils::CoordinateHelper::calculateGravity(state.posEcf);
        const auto acc_ecf                  = (total_acc_ecf - gravity_accel).eval();
        const auto acc_nue                  = ModelDevelop::Utils::CoordinateHelper::ecefToNueAcceleration(acc_ecf, lla.x(), lla.y());

        imu_info.imu_acc_body = ModelDevelop::Utils::CoordinateHelper::nueToBodyAcceleration(acc_nue, state.qbn);

        imu_info.imu_w_xyz_body = state.wnb_b;
        imu_info.imu_ypr        = ModelDevelop::Utils::CoordinateHelper::quaternionToEuler231(state.qbn) / 57.3;
        return imu_info;
    }

// endregion

// region Get/Set选择器
// endregion

// region Private Methods
// endregion
}
#undef PRETTY_FILE_NAME